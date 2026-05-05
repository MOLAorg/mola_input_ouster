/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   OusterDirectInput.cpp
 * @brief  RawDataSource directly from an Ouster LiDAR via the Ouster SDK
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

/** \defgroup mola_input_ouster_grp mola_input_ouster
 * RawDataSource from an Ouster LiDAR sensor using the native Ouster C++ SDK.
 */

#include <mola_input_ouster/OusterDirectInput.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/initializer.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/filesystem.h>

// Ouster SDK (0.16+):
#include <ouster/cartesian.h>
#include <ouster/client.h>
#include <ouster/image_processing.h>
#include <ouster/lidar_scan.h>
#include <ouster/os_pcap.h>
#include <ouster/osf/meta_lidar_sensor.h>
#include <ouster/osf/reader.h>
#include <ouster/osf/stream_lidar_scan.h>
#include <ouster/sensor_packet_source.h>
#include <ouster/types.h>
#include <ouster/xyzlut.h>

// Convenience aliases for the SDK 0.16 namespace structure
namespace sc   = ouster::sdk::core;
namespace ss   = ouster::sdk::sensor;
namespace spc  = ouster::sdk::pcap;
namespace sosf = ouster::sdk::osf;

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(OusterDirectInput, RawDataSourceBase, mola)

MRPT_INITIALIZER(do_register_OusterDirectInput)  // NOLINT(misc-use-anonymous-namespace)
{
  MOLA_REGISTER_MODULE(OusterDirectInput);
}

// ============================================================================
// Anonymous namespace for file-local helpers
// ============================================================================
namespace
{

mrpt::poses::CPose3D parsePoseString(const std::string& s)
{
  if (s.empty())
  {
    return mrpt::poses::CPose3D();
  }

  std::istringstream ss(s);
  double             x = 0, y = 0, z = 0, yaw_deg = 0, pitch_deg = 0, roll_deg = 0;
  ss >> x >> y >> z >> yaw_deg >> pitch_deg >> roll_deg;

  return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
      x, y, z, mrpt::DEG2RAD(yaw_deg), mrpt::DEG2RAD(pitch_deg), mrpt::DEG2RAD(roll_deg));
}

mrpt::Clock::time_point ousterTsToMrpt(uint64_t nsec)
{
  // Ouster timestamps are nanoseconds since Unix epoch (PTP) or since sensor
  // boot (internal clock). mrpt::Clock uses the Windows FILETIME epoch
  // (1601-01-01, 100ns ticks); fromDouble() handles that offset correctly.
  return mrpt::Clock::fromDouble(static_cast<double>(nsec) * 1e-9);
}

}  // namespace

// ============================================================================
// Anonymous namespace: Ouster mat4d → CPose3D conversion
// ============================================================================
namespace
{

/** Convert an Ouster mat4d (4×4, column-major Eigen, translations in mm)
 *  to an mrpt::poses::CPose3D (translations in meters). */
mrpt::poses::CPose3D mat4dToPose(const Eigen::Matrix<double, 4, 4>& m)
{
  // Ouster stores these in row-major in JSON, but the SDK parses them
  // into Eigen which is column-major by default. The mat4d typedef uses
  // Eigen::DontAlign but default (column-major) storage.
  // Extract rotation and translation.
  mrpt::math::CMatrixDouble44 hm;
  for (int r = 0; r < 4; ++r)
  {
    for (int c = 0; c < 4; ++c)
    {
      hm(r, c) = m(r, c);
    }
  }

  // Translation is in millimeters in the sensor firmware — convert to meters
  constexpr double MM_TO_M = 0.001;
  hm(0, 3) *= MM_TO_M;
  hm(1, 3) *= MM_TO_M;
  hm(2, 3) *= MM_TO_M;

  return mrpt::poses::CPose3D(hm);
}

}  // namespace

// ============================================================================
// PIMPL: Holds all Ouster SDK types that we want to keep out of the header
// ============================================================================
struct OusterDirectInput::OusterState
{
  // Live sensor packet source (null in pcap mode)
  std::unique_ptr<ss::SensorPacketSource> packetSource;

  // Metadata (populated in both modes)
  sc::SensorInfo info;

  // Packet format (derived from SensorInfo); shared_ptr because Packet::format
  // is also a shared_ptr and both packets hold a reference to the same object.
  std::shared_ptr<sc::PacketFormat> pf;

  // Batching packets into full scans
  std::unique_ptr<sc::ScanBatcher> batcher;
  std::unique_ptr<sc::LidarScan>   scan;

  // Lookup table for converting range -> XYZ
  sc::XYZLut xyzLut;

  // Typed packet objects (SDK 0.16: ScanBatcher takes Packet&, not uint8_t*)
  sc::LidarPacket lidarPkt;
  sc::ImuPacket   imuPkt;

  // Scan dimensions
  int w = 0;  // columns per revolution
  int h = 0;  // pixels per column (channels)

  // PCAP replay handle (null in live mode)
  std::shared_ptr<spc::PlaybackHandle> pcapHandle;
  int                                  pcapLidarPort = 0;
  int                                  pcapImuPort   = 0;

  // Reusable scratch buffer for skipping unknown packets
  std::vector<uint8_t> scratchBuf;

  // OSF replay state (null unless in OSF mode)
  std::unique_ptr<sosf::Reader>                osfReader;
  std::unique_ptr<sosf::MessagesStreamingIter> osfIter;
  std::unique_ptr<sosf::MessagesStreamingIter> osfEnd;

  // Stateful auto-exposure for float16 RGB fields (SDK v0.16.2+).
  // Kept across scans so it converges on stable exposure over time.
  sc::image::AutoExposure rgbAutoExposure;
};

// ============================================================================
// Construction / destruction
// ============================================================================
OusterDirectInput::OusterDirectInput() { this->setLoggerName("OusterDirectInput"); }

OusterDirectInput::~OusterDirectInput()
{
  // onQuit() should have been called already, but guard just in case:
  receiverRunning_ = false;
  if (receiverThread_.joinable())
  {
    receiverThread_.join();
  }
}

void OusterDirectInput::onQuit()
{
  MRPT_LOG_DEBUG("OusterDirectInput::onQuit() called.");

  // Stop receiver thread before any resources are destroyed
  receiverRunning_ = false;
  if (receiverThread_.joinable())
  {
    receiverThread_.join();
  }

  // Clean up PCAP handle
  if (ousterState_ && ousterState_->pcapHandle)
  {
    spc::replay_uninitialize(*(ousterState_->pcapHandle));
    ousterState_->pcapHandle.reset();
  }
}

// ============================================================================
// initialize_rds
// ============================================================================
void OusterDirectInput::initialize_rds(const Yaml& c)
{
  using namespace std::string_literals;

  MRPT_START
  ProfilerEntry tle(profiler_, "initialize");

  ENSURE_YAML_ENTRY_EXISTS(c, "params");
  const auto cfg = c["params"];
  MRPT_LOG_DEBUG_STREAM("Initializing with these params:\n" << cfg);

  // --- Parse YAML parameters ---
  if (cfg.has("sensor_hostname"))
  {
    params_.sensor_hostname = cfg["sensor_hostname"].as<std::string>();
  }
  if (cfg.has("udp_dest"))
  {
    params_.udp_dest = cfg["udp_dest"].as<std::string>();
  }
  if (cfg.has("lidar_port"))
  {
    params_.lidar_port = cfg["lidar_port"].as<int>();
  }
  if (cfg.has("imu_port"))
  {
    params_.imu_port = cfg["imu_port"].as<int>();
  }
  if (cfg.has("pcap_file"))
  {
    params_.pcap_file = cfg["pcap_file"].as<std::string>();
  }
  if (cfg.has("metadata_json"))
  {
    params_.metadata_json = cfg["metadata_json"].as<std::string>();
  }
  if (cfg.has("osf_file"))
  {
    params_.osf_file = cfg["osf_file"].as<std::string>();
  }
  if (cfg.has("time_warp_scale"))
  {
    params_.time_warp_scale = cfg["time_warp_scale"].as<double>();
  }
  if (cfg.has("lidar_mode"))
  {
    params_.lidar_mode = cfg["lidar_mode"].as<std::string>();
  }
  if (cfg.has("timestamp_mode"))
  {
    params_.timestamp_mode = cfg["timestamp_mode"].as<std::string>();
  }
  if (cfg.has("lidar_sensor_label"))
  {
    params_.lidar_sensor_label = cfg["lidar_sensor_label"].as<std::string>();
  }
  if (cfg.has("imu_sensor_label"))
  {
    params_.imu_sensor_label = cfg["imu_sensor_label"].as<std::string>();
  }
  if (cfg.has("sensor_mounting_pose"))
  {
    params_.sensor_mounting_pose = parsePoseString(cfg["sensor_mounting_pose"].as<std::string>());
  }
  if (cfg.has("lidar_sensor_pose"))
  {
    params_.lidar_sensor_pose_override =
        parsePoseString(cfg["lidar_sensor_pose"].as<std::string>());
  }
  if (cfg.has("imu_sensor_pose"))
  {
    params_.imu_sensor_pose_override = parsePoseString(cfg["imu_sensor_pose"].as<std::string>());
  }

  // --- Validate ---
  ASSERTMSG_(
      !params_.sensor_hostname.empty() || !params_.pcap_file.empty() || !params_.osf_file.empty(),
      "One of 'sensor_hostname' (live), 'pcap_file' (PCAP replay), or "
      "'osf_file' (OSF replay) must be provided.");

  ASSERTMSG_(
      (params_.sensor_hostname.empty() ? 0 : 1) + (params_.pcap_file.empty() ? 0 : 1) +
              (params_.osf_file.empty() ? 0 : 1) <=
          1,
      "Only one of 'sensor_hostname', 'pcap_file', 'osf_file' may be set.");

  if (!params_.pcap_file.empty())
  {
    ASSERTMSG_(
        !params_.metadata_json.empty(), "'metadata_json' is required when using 'pcap_file'.");
    ASSERTMSG_(
        mrpt::system::fileExists(params_.pcap_file),
        "pcap_file not found: '"s + params_.pcap_file + "'"s);
    ASSERTMSG_(
        mrpt::system::fileExists(params_.metadata_json),
        "metadata_json not found: '"s + params_.metadata_json + "'"s);
  }

  if (!params_.osf_file.empty())
  {
    ASSERTMSG_(
        mrpt::system::fileExists(params_.osf_file),
        "osf_file not found: '"s + params_.osf_file + "'"s);
  }

  // --- Create Ouster state ---
  ousterState_ = std::make_unique<OusterState>();

  if (isLiveMode())
  {
    initLiveMode();
  }
  else if (isOsfMode())
  {
    initOsfMode();
  }
  else
  {
    initPcapMode();
  }

  setupOusterFromInfo();

  // ---- Start receiver thread for live mode ----
  if (isLiveMode())
  {
    receiverRunning_ = true;
    receiverThread_  = std::thread(&OusterDirectInput::receiverThreadFunc, this);
  }

  MRPT_END
}

// ============================================================================
// initLiveMode: Connect to a live Ouster sensor
// ============================================================================
void OusterDirectInput::initLiveMode()
{
  using namespace std::string_literals;

  MRPT_LOG_INFO_STREAM("Connecting to Ouster sensor at '" << params_.sensor_hostname << "' ...");

  auto ld_mode = sc::lidar_mode_of_string(params_.lidar_mode);
  auto ts_mode = sc::timestamp_mode_of_string(params_.timestamp_mode);

  ASSERTMSG_(ld_mode.has_value(), "Invalid lidar_mode: '"s + params_.lidar_mode + "'"s);
  ASSERTMSG_(
      ts_mode != sc::TimestampMode::UNSPECIFIED,
      "Invalid timestamp_mode: '"s + params_.timestamp_mode + "'"s);

  // Build a SensorConfig with the desired resolution / timestamp mode
  sc::SensorConfig cfg;
  cfg.lidar_mode     = ld_mode;
  cfg.timestamp_mode = ts_mode;
  if (!params_.udp_dest.empty()) cfg.udp_dest = params_.udp_dest;
  if (params_.lidar_port) cfg.udp_port_lidar = static_cast<uint16_t>(params_.lidar_port);
  if (params_.imu_port) cfg.udp_port_imu = static_cast<uint16_t>(params_.imu_port);

  // Create a SensorPacketSource — replaces deprecated init_client
  ousterState_->packetSource = std::make_unique<ss::SensorPacketSource>(
      params_.sensor_hostname,
      [&cfg, this](ss::SensorPacketSourceOptions& opts)
      {
        opts.sensor_config = {cfg};
        if (!params_.udp_dest.empty()) opts.no_auto_udp_dest = true;
      });

  // Retrieve metadata from the connected sensor
  const auto& infos = ousterState_->packetSource->sensor_info();
  ASSERTMSG_(
      !infos.empty() && infos[0],
      "Failed to retrieve metadata from Ouster sensor at '"s + params_.sensor_hostname + "'"s);
  ousterState_->info = *(infos[0]);

  MRPT_LOG_INFO_STREAM(
      "Connected to Ouster sensor. Product: " << ousterState_->info.prod_line
                                              << "  SN: " << ousterState_->info.sn
                                              << "  FW: " << ousterState_->info.fw_rev);
}

// ============================================================================
// initPcapMode: Open a PCAP file + metadata JSON for replay
// ============================================================================
void OusterDirectInput::initPcapMode()
{
  MRPT_LOG_INFO_STREAM("Opening Ouster PCAP: " << params_.pcap_file);

  // Load metadata from JSON file
  std::ifstream ifs(params_.metadata_json);
  ASSERTMSG_(ifs.good(), "Cannot open metadata JSON file.");
  std::string metadata_str((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

  ousterState_->info = sc::SensorInfo(metadata_str);

  MRPT_LOG_INFO_STREAM(
      "Loaded Ouster metadata. Product: " << ousterState_->info.prod_line
                                          << "  SN: " << ousterState_->info.sn);

  // Open pcap file for stepwise playback
  ousterState_->pcapHandle = spc::replay_initialize(params_.pcap_file);
  ASSERTMSG_(ousterState_->pcapHandle, "Failed to open PCAP file.");

  // Determine the UDP ports used for LiDAR and IMU data.
  // These come from the sensor metadata (config section).
  const auto& config          = ousterState_->info.config;
  ousterState_->pcapLidarPort = config.udp_port_lidar.value_or(7502);
  ousterState_->pcapImuPort   = config.udp_port_imu.value_or(7503);

  MRPT_LOG_INFO_FMT(
      "PCAP using lidar_port=%d, imu_port=%d", ousterState_->pcapLidarPort,
      ousterState_->pcapImuPort);
}

// ============================================================================
// initOsfMode: Open an OSF file for replay
// ============================================================================
void OusterDirectInput::initOsfMode()
{
  MRPT_LOG_INFO_STREAM("Opening Ouster OSF: " << params_.osf_file);

  ousterState_->osfReader = std::make_unique<sosf::Reader>(params_.osf_file);

  // Extract SensorInfo from the OSF metadata store
  auto lidarSensorMeta = ousterState_->osfReader->meta_store().get<sosf::LidarSensor>();
  ASSERTMSG_(lidarSensorMeta, "OSF file contains no LidarSensor metadata entry.");

  ousterState_->info = lidarSensorMeta->info();

  MRPT_LOG_INFO_STREAM(
      "Loaded OSF metadata. Product: " << ousterState_->info.prod_line
                                       << "  SN: " << ousterState_->info.sn);

  // Create the streaming range and store begin/end iterators
  auto range            = ousterState_->osfReader->messages();
  ousterState_->osfIter = std::make_unique<sosf::MessagesStreamingIter>(range.begin());
  ousterState_->osfEnd  = std::make_unique<sosf::MessagesStreamingIter>(range.end());
}

// ============================================================================
// Setup batcher, XYZLut, buffers from sensor_info
// ============================================================================
void OusterDirectInput::setupOusterFromInfo()
{
  const auto& info = ousterState_->info;

  ousterState_->w = info.format.columns_per_frame;
  ousterState_->h = info.format.pixels_per_column;

  MRPT_LOG_INFO_FMT("Ouster scan format: %d x %d (cols x rows)", ousterState_->w, ousterState_->h);

  // Packet format
  ousterState_->pf = std::make_shared<sc::PacketFormat>(sc::get_format(info));

  // Scan batcher
  ousterState_->batcher = std::make_unique<sc::ScanBatcher>(info);

  // Allocate a LidarScan
  ousterState_->scan = std::make_unique<sc::LidarScan>(
      ousterState_->w, ousterState_->h, info.format.udp_profile_lidar);

  // XYZ lookup table
  ousterState_->xyzLut = sc::make_xyz_lut(info, /*use_extrinsics=*/false);

  // Allocate typed packet objects; share the PacketFormat with each packet
  // so ScanBatcher can validate them (SDK 0.16 requirement).
  const auto& pf                = *(ousterState_->pf);
  ousterState_->lidarPkt        = sc::LidarPacket(pf.lidar_packet_size);
  ousterState_->imuPkt          = sc::ImuPacket(pf.imu_packet_size);
  ousterState_->lidarPkt.format = ousterState_->pf;
  ousterState_->imuPkt.format   = ousterState_->pf;

  // ---- Resolve observation sensorPose from intrinsic transforms ----
  //
  // In MRPT/MOLA, CObservation::sensorPose is the SE(3) pose of the
  // sensor's own coordinate frame w.r.t. the vehicle frame (base_link).
  // Point coordinates inside the observation are expressed in the
  // sensor's own frame (lidar frame for point clouds, IMU frame for
  // IMU readings). This is consistent with how mrpt::ros2bridge and
  // BridgeROS2 work: they query /tf for base_link → sensor_frame_id
  // and set that as sensorPose.
  //
  // The Ouster sensor stores factory-calibrated 4×4 transforms (mm):
  //   lidar_to_sensor_transform: Lidar frame → Sensor housing frame
  //   imu_to_sensor_transform:   IMU frame   → Sensor housing frame
  //
  // The user provides sensor_mounting_pose = pose of the sensor housing
  // on the vehicle (base_link → os_sensor).
  //
  // We compose to get the full chain to each sensor's native frame:
  //   resolved_lidar_pose = mounting ∘ lidar_to_sensor
  //                       = base_link → os_sensor → os_lidar
  //   resolved_imu_pose   = mounting ∘ imu_to_sensor
  //                       = base_link → os_sensor → os_imu
  //
  // This matches the ouster-ros TF tree exactly. Points in
  // CObservationPointCloud::pointcloud remain in the lidar frame;
  // IMU readings in CObservationIMU remain in the IMU frame.

  if (params_.lidar_sensor_pose_override.has_value())
  {
    resolvedLidarPose_ = params_.lidar_sensor_pose_override.value();
    MRPT_LOG_INFO_STREAM(
        "Using manual lidar_sensor_pose override: " << resolvedLidarPose_.asString());
  }
  else
  {
    const auto lidarIntrinsic = mat4dToPose(info.lidar_to_sensor_transform);
    resolvedLidarPose_        = params_.sensor_mounting_pose + lidarIntrinsic;
    MRPT_LOG_INFO_STREAM(
        "Lidar frame pose on vehicle (base_link -> os_lidar): "
        << resolvedLidarPose_.asString()
        << "\n  mounting (base_link->os_sensor): " << params_.sensor_mounting_pose.asString()
        << "\n  intrinsic (os_sensor->os_lidar): " << lidarIntrinsic.asString());
  }

  if (params_.imu_sensor_pose_override.has_value())
  {
    resolvedImuPose_ = params_.imu_sensor_pose_override.value();
    MRPT_LOG_INFO_STREAM("Using manual imu_sensor_pose override: " << resolvedImuPose_.asString());
  }
  else
  {
    const auto imuIntrinsic = mat4dToPose(info.imu_to_sensor_transform);
    resolvedImuPose_        = params_.sensor_mounting_pose + imuIntrinsic;
    MRPT_LOG_INFO_STREAM(
        "IMU frame pose on vehicle (base_link -> os_imu): "
        << resolvedImuPose_.asString()
        << "\n  mounting (base_link->os_sensor): " << params_.sensor_mounting_pose.asString()
        << "\n  intrinsic (os_sensor->os_imu): " << imuIntrinsic.asString());
  }
}

// ============================================================================
// scanToObservation: Convert Ouster LidarScan -> CObservationPointCloud
//
// Point coordinates are in the Ouster *Sensor Coordinate Frame*
// (make_xyz_lut(info) bakes lidar_to_sensor_transform into the LUT,
// so cartesian() output is already in the sensor housing frame, which
// is the standard frame used by ouster-ros for /ouster/points).
//
// sensorPose is set to resolvedLidarPose_ = base_link → os_sensor,
// (or the full chain base_link → os_sensor → os_lidar when intrinsic ≠ I).
// This is consistent with how BridgeROS2 / mrpt::ros2bridge sets sensorPose.
// ============================================================================
mrpt::obs::CObservationPointCloud::Ptr OusterDirectInput::scanToObservation(
    const sc::LidarScan& scan)
{
  const ProfilerEntry tleg(profiler_, "scanToObservation");

  const int  W        = ousterState_->w;
  const int  H        = ousterState_->h;
  const auto totalPts = static_cast<std::size_t>(W) * static_cast<std::size_t>(H);

  // Convert range data to XYZ using the precomputed lookup table.
  // cartesianT() returns an Eigen::Array<double, -1, 3> of shape (W*H, 3).
  auto cloud =
      sc::cartesianT<double>(scan, ousterState_->xyzLut.direction, ousterState_->xyzLut.offset);

  // Get the range field to filter out invalid (zero-range) points.
  // range is a 2D Eigen array of shape (H, W).
  auto range = scan.field<uint32_t>(sc::ChanField::RANGE);

  // Detect which optional fields are present in this scan
  const bool hasSig  = scan.has_field(sc::ChanField::SIGNAL);
  const bool hasRefl = scan.has_field(sc::ChanField::REFLECTIVITY);
  const bool hasRGB  = scan.has_field(sc::ChanField::RGB);

  // Convert a float16 bit pattern to float32.
  auto f16_to_f32 = [](uint16_t h) -> float
  {
    uint32_t s = (h >> 15u) & 1u;
    uint32_t e = (h >> 10u) & 0x1fu;
    uint32_t m = h & 0x3ffu;
    uint32_t bits;
    if (e == 0)
      bits = (s << 31u) | ((m == 0) ? 0u : ((113u << 23u) | (m << 13u)));
    else if (e == 31u)
      bits = (s << 31u) | 0x7f800000u | (m << 13u);
    else
      bits = (s << 31u) | ((e + 112u) << 23u) | (m << 13u);
    float f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
  };

  // Helper: read a scalar pixel field as float regardless of its underlying
  // integer type (UINT8 / UINT16 / UINT32 / FLOAT16).  Returns 0 for unknown types.
  auto readPixelFloat = [&f16_to_f32](
                            const sc::LidarScan& s, const std::string& name, Eigen::Index r,
                            Eigen::Index c) -> float
  {
    const auto& f = s.field(name);
    switch (f.tag())
    {
      case sc::ChanFieldType::UINT8:
        return static_cast<float>(f.get<uint8_t>()[r * static_cast<Eigen::Index>(s.w) + c]);
      case sc::ChanFieldType::UINT16:
        return static_cast<float>(f.get<uint16_t>()[r * static_cast<Eigen::Index>(s.w) + c]);
      case sc::ChanFieldType::UINT32:
        return static_cast<float>(f.get<uint32_t>()[r * static_cast<Eigen::Index>(s.w) + c]);
      case sc::ChanFieldType::FLOAT16:
        return f16_to_f32(f.get<sc::float16_t>()[r * static_cast<Eigen::Index>(s.w) + c].data);
      default:
        return 0.f;
    }
  };

  // RGB raw bytes — only valid when hasRGB; layout [H, W, 3] row-major.
  // pixel at flat index idx = row*W + col → bytes rgbRaw[idx*3 + {0,1,2}].
  // SDK v0.16.2 introduced FLOAT16 RGB for the new RNG19_RFL8_SIG16_NIR16_RGB16
  // profiles. Those values are raw HDR (linear, unbounded); we apply
  // AutoExposure (percentile contrast-stretch → [0,1]) before scaling to uint8.
  // Older profiles use UINT8 packed bytes.
  const uint8_t*       rgbRaw = nullptr;
  std::vector<uint8_t> rgbBuf;
  if (hasRGB)
  {
    const auto& rgbField = scan.field(sc::ChanField::RGB);
    if (rgbField.tag() == sc::ChanFieldType::FLOAT16)
    {
      // Build TensorMaps over the raw float16 field and a float scratch buffer.
      const auto* f16ptr  = rgbField.get<sc::float16_t>();
      const auto  nElems  = static_cast<Eigen::Index>(H) * static_cast<Eigen::Index>(W) * 3;
      std::vector<float> floatBuf(static_cast<std::size_t>(nElems));

      // TensorMap shapes: [H, W, 3] row-major (RowMajor Tensor).
      using F16TensorMap = Eigen::TensorMap<const sc::rgb_img_t<sc::float16_t>>;
      using F32TensorMap = Eigen::TensorMap<sc::rgb_img_t<float>>;
      F16TensorMap inputMap(f16ptr, static_cast<Eigen::Index>(H),
                                    static_cast<Eigen::Index>(W), 3);
      F32TensorMap outputMap(floatBuf.data(), static_cast<Eigen::Index>(H),
                                              static_cast<Eigen::Index>(W), 3);

      ousterState_->rgbAutoExposure.update(inputMap, outputMap);

      rgbBuf.resize(static_cast<std::size_t>(nElems));
      for (std::size_t i = 0; i < static_cast<std::size_t>(nElems); ++i)
        rgbBuf[i] = static_cast<uint8_t>(std::clamp(floatBuf[i] * 255.f, 0.f, 255.f));
      rgbRaw = rgbBuf.data();
    }
    else
    {
      rgbRaw = rgbField.get<uint8_t>();
    }
  }

  // Build MRPT point cloud — use CGenericPointsMap for arbitrary extra fields
  auto pts = mrpt::maps::CGenericPointsMap::Create();
  pts->reserve(totalPts);

  if (hasSig) pts->registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY);
  if (hasRefl) pts->registerField_float("reflectivity");
  if (hasRGB)
  {
    pts->registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8);
    pts->registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gu8);
    pts->registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bu8);
  }
  pts->registerField_float("t");

  const auto& timestamps = scan.timestamp();

  // Find the first non-zero column timestamp to use as the scan origin.
  uint64_t firstTs = 0;
  for (Eigen::Index c = 0; c < timestamps.size(); ++c)
  {
    if (timestamps(c) != 0)
    {
      firstTs = static_cast<uint64_t>(timestamps(c));
      break;
    }
  }

  for (std::size_t col = 0; col < static_cast<std::size_t>(W); ++col)
  {
    const auto     c      = static_cast<Eigen::Index>(col);
    const uint64_t colTs  = static_cast<uint64_t>(timestamps(c));
    const float    t_secs = (colTs >= firstTs) ? static_cast<float>((colTs - firstTs) * 1e-9) : 0.f;

    for (std::size_t row = 0; row < static_cast<std::size_t>(H); ++row)
    {
      const auto r = static_cast<Eigen::Index>(row);

      if (range(r, c) == 0) continue;  // invalid point

      // cartesian() output is row-major (H, W):
      //   pixel at (row, col) -> flat index = row * W + col
      const auto idx = static_cast<Eigen::Index>(row * static_cast<std::size_t>(W) + col);
      pts->insertPointFast(
          static_cast<float>(cloud(idx, 0)), static_cast<float>(cloud(idx, 1)),
          static_cast<float>(cloud(idx, 2)));

      pts->insertPointField_float("t", t_secs);
      if (hasSig)
        pts->insertPointField_float(
            mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY,
            readPixelFloat(scan, sc::ChanField::SIGNAL, r, c));
      if (hasRefl)
        pts->insertPointField_float(
            "reflectivity", readPixelFloat(scan, sc::ChanField::REFLECTIVITY, r, c));
      if (hasRGB && rgbRaw)
      {
        const uint8_t* px = rgbRaw + static_cast<std::size_t>(idx) * 3;
        pts->insertPointField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8, px[0]);
        pts->insertPointField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gu8, px[1]);
        pts->insertPointField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bu8, px[2]);
      }
    }
  }

  // Scan-level timestamp: use the first column timestamp (same origin as "t" fields).
  const uint64_t scanTsNsec = firstTs;

  auto obs         = mrpt::obs::CObservationPointCloud::Create();
  obs->pointcloud  = std::move(pts);
  obs->sensorLabel = params_.lidar_sensor_label;
  obs->sensorPose  = resolvedLidarPose_;
  obs->timestamp   = ousterTsToMrpt(scanTsNsec);

  return obs;
}

// ============================================================================
// imuToObservation: Convert Ouster IMU packet -> CObservationIMU
//
// Accelerations and angular velocities are in the Ouster *IMU frame*.
// sensorPose is set to resolvedImuPose_ = base_link → os_imu,
// so downstream consumers know where the IMU frame sits on the vehicle.
//
// Unit conversion depends on the IMU UDP profile:
//   LEGACY profile:     imu_la_{x,y,z} in g      → multiply by 9.80665 → m/s²
//                       imu_av_{x,y,z} in deg/s  → DEG2RAD              → rad/s
//   Non-legacy profile: imu_la_{x,y,z} already in m/s²  (no conversion)
//                       imu_av_{x,y,z} already in rad/s (no conversion)
// This mirrors the logic in the Ouster SDK's ImuPacket::accel() / gyro().
// ============================================================================
mrpt::obs::CObservationIMU::Ptr OusterDirectInput::imuToObservation(const uint8_t* buf)
{
  const ProfilerEntry tleg(profiler_, "imuToObservation");

  const auto& pf = *(ousterState_->pf);

  const uint64_t tsNsec = pf.imu_gyro_ts(buf);

  double laX, laY, laZ, avX, avY, avZ;
  if (pf.udp_profile_imu == sc::UDPProfileIMU::LEGACY)
  {
    constexpr double G_TO_MS2 = 9.80665;
    laX                       = static_cast<double>(pf.imu_la_x(buf)) * G_TO_MS2;
    laY                       = static_cast<double>(pf.imu_la_y(buf)) * G_TO_MS2;
    laZ                       = static_cast<double>(pf.imu_la_z(buf)) * G_TO_MS2;
    avX                       = mrpt::DEG2RAD(static_cast<double>(pf.imu_av_x(buf)));
    avY                       = mrpt::DEG2RAD(static_cast<double>(pf.imu_av_y(buf)));
    avZ                       = mrpt::DEG2RAD(static_cast<double>(pf.imu_av_z(buf)));
  }
  else
  {
    // Non-legacy profiles already report m/s² and rad/s
    laX = static_cast<double>(pf.imu_la_x(buf));
    laY = static_cast<double>(pf.imu_la_y(buf));
    laZ = static_cast<double>(pf.imu_la_z(buf));
    avX = static_cast<double>(pf.imu_av_x(buf));
    avY = static_cast<double>(pf.imu_av_y(buf));
    avZ = static_cast<double>(pf.imu_av_z(buf));
  }

  auto obs         = mrpt::obs::CObservationIMU::Create();
  obs->sensorLabel = params_.imu_sensor_label;
  obs->sensorPose  = resolvedImuPose_;
  obs->timestamp   = ousterTsToMrpt(tsNsec);

  obs->set(mrpt::obs::IMU_X_ACC, laX);
  obs->set(mrpt::obs::IMU_Y_ACC, laY);
  obs->set(mrpt::obs::IMU_Z_ACC, laZ);
  obs->set(mrpt::obs::IMU_WX, avX);
  obs->set(mrpt::obs::IMU_WY, avY);
  obs->set(mrpt::obs::IMU_WZ, avZ);

  return obs;
}

// ============================================================================
// receiverThreadFunc: Background thread for live sensor data reception
// ============================================================================
void OusterDirectInput::receiverThreadFunc()
{
  MRPT_LOG_INFO("Ouster receiver thread started.");

  auto& src     = *(ousterState_->packetSource);
  auto& batcher = *(ousterState_->batcher);
  auto& scan    = *(ousterState_->scan);

  while (receiverRunning_ && !requestedShutdown())
  {
    try
    {
      // get_packet blocks for up to 1 s, then returns a POLL_TIMEOUT event.
      const auto event = src.get_packet(/*timeout_sec=*/1.0);

      if (event.type == ss::ClientEvent::ERR)
      {
        MRPT_LOG_ERROR("Ouster SensorPacketSource returned ERR.");
        break;
      }
      if (event.type == ss::ClientEvent::EXIT)
      {
        MRPT_LOG_INFO("Ouster SensorPacketSource returned EXIT.");
        break;
      }
      if (event.type == ss::ClientEvent::POLL_TIMEOUT)
      {
        continue;  // normal timeout — re-check receiverRunning_
      }

      // event.type == PACKET
      auto& pkt = const_cast<ss::ClientEvent&>(event).packet();
      if (pkt.type() == sc::PacketType::Lidar)
      {
        auto& lidarPkt = static_cast<sc::LidarPacket&>(pkt);
        // Feed packet to batcher; returns true when a full scan is assembled.
        if (batcher(lidarPkt, scan))
        {
          auto obs = scanToObservation(scan);
          if (obs && obs->pointcloud && obs->pointcloud->size() > 0)
          {
            sendObservationsToFrontEnds(obs);
          }
        }
      }
      else if (pkt.type() == sc::PacketType::Imu)
      {
        auto& imuPkt = static_cast<sc::ImuPacket&>(pkt);
        auto  obs    = imuToObservation(imuPkt.buf.data());
        if (obs)
        {
          sendObservationsToFrontEnds(obs);
        }
      }
    }
    catch (const std::exception& e)
    {
      MRPT_LOG_ERROR_STREAM("Exception in Ouster receiver thread:\n" << mrpt::exception_to_str(e));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  MRPT_LOG_INFO("Ouster receiver thread finished.");
}

// ============================================================================
// spinOnce
// ============================================================================
void OusterDirectInput::spinOnce()
{
  if (isLiveMode())
  {
    // In live mode, the receiver thread handles data.
    // spinOnce() is used only for diagnostics.
    if (module_is_time_to_publish_diagnostics())
    {
      DiagnosticsOutput diag;
      diag.timestamp = mrpt::Clock::now();
      diag.label     = "OusterDirectInput:alive";
      diag.value     = receiverRunning_.load();
      module_publish_diagnostics(diag);
    }
  }
  else if (isOsfMode())
  {
    osfSpinOnce();
  }
  else
  {
    // PCAP replay: drive from spinOnce()
    pcapSpinOnce();
  }
}

// ============================================================================
// pcapSpinOnce: Drive PCAP replay at the configured pace
//
// Uses the ouster_pcap stepwise API:
//   next_packet_info(handle, info)  — peek at next packet's metadata
//   read_packet(handle, buf, size)  — read the packet payload
// Packets are distinguished by destination UDP port (lidar vs imu).
// ============================================================================
void OusterDirectInput::pcapSpinOnce()
{
  if (!ousterState_ || !ousterState_->pcapHandle)
  {
    return;
  }

  auto& handle  = *(ousterState_->pcapHandle);
  auto& batcher = *(ousterState_->batcher);
  auto& scan    = *(ousterState_->scan);

  const int lidarPort = ousterState_->pcapLidarPort;
  const int imuPort   = ousterState_->pcapImuPort;

  // Read packets from PCAP until we assemble one full LiDAR scan,
  // then pace according to time_warp_scale.
  bool gotScan = false;
  while (!gotScan && !requestedShutdown())
  {
    spc::PacketInfo pktInfo;
    if (!spc::next_packet_info(handle, pktInfo))
    {
      // End of file
      MRPT_LOG_INFO("End of PCAP file reached.");
      onDatasetPlaybackEnds();
      return;
    }

    const int  dstPort     = pktInfo.dst_port;
    const auto payloadSize = static_cast<std::size_t>(pktInfo.payload_size);

    if (dstPort == lidarPort && payloadSize <= ousterState_->lidarPkt.buf.size())
    {
      // Read lidar packet
      const auto nRead = spc::read_packet(
          handle, ousterState_->lidarPkt.buf.data(), ousterState_->lidarPkt.buf.size());

      if (nRead > 0)
      {
        if (batcher(ousterState_->lidarPkt, scan))
        {
          auto obs = scanToObservation(scan);
          if (obs && obs->pointcloud && obs->pointcloud->size() > 0)
          {
            // Pace the replay
            paceReplay(obs->timestamp);
            sendObservationsToFrontEnds(obs);
            gotScan = true;
          }
        }
      }
    }
    else if (dstPort == imuPort && payloadSize <= ousterState_->imuPkt.buf.size())
    {
      // Read IMU packet
      const auto nRead = spc::read_packet(
          handle, ousterState_->imuPkt.buf.data(), ousterState_->imuPkt.buf.size());

      if (nRead > 0)
      {
        auto obs = imuToObservation(ousterState_->imuPkt.buf.data());
        if (obs)
        {
          sendObservationsToFrontEnds(obs);
        }
      }
    }
    else
    {
      // Unknown/unrelated packet — skip it
      if (ousterState_->scratchBuf.size() < payloadSize + 1)
      {
        ousterState_->scratchBuf.resize(payloadSize + 1);
      }
      spc::read_packet(handle, ousterState_->scratchBuf.data(), ousterState_->scratchBuf.size());
    }
  }
}

// ============================================================================
// osfSpinOnce: Drive OSF replay at the configured pace
//
// Advances through the OSF message stream until one full LidarScan is
// emitted, then paces according to time_warp_scale.
// ============================================================================
void OusterDirectInput::osfSpinOnce()
{
  if (!ousterState_ || !ousterState_->osfIter || !ousterState_->osfEnd)
  {
    return;
  }

  auto& it  = *(ousterState_->osfIter);
  auto& end = *(ousterState_->osfEnd);

  bool gotScan = false;
  while (!gotScan && it != end && !requestedShutdown())
  {
    auto msg = *it;
    ++it;

    if (msg.is<sosf::LidarScanStream>())
    {
      auto scan = msg.decode_msg<sosf::LidarScanStream>();
      if (scan)
      {
        // Emit IMU observations from embedded IMU fields (ACCEL32_GYRO32_NMEA
        // profile stores multiple IMU samples per LidarScan).
        // IMU_ACC is in m/s² and IMU_GYRO is in rad/s (non-legacy profile).
        if (scan->has_field(sc::ChanField::IMU_TIMESTAMP) &&
            scan->has_field(sc::ChanField::IMU_ACC) && scan->has_field(sc::ChanField::IMU_GYRO))
        {
          const sc::ArrayView1<uint64_t> imuTs   = scan->field(sc::ChanField::IMU_TIMESTAMP);
          const sc::ArrayView2<float>    imuAcc  = scan->field(sc::ChanField::IMU_ACC);
          const sc::ArrayView2<float>    imuGyro = scan->field(sc::ChanField::IMU_GYRO);

          for (std::size_t i = 0; i < imuTs.shape[0]; ++i)
          {
            auto imuObs         = mrpt::obs::CObservationIMU::Create();
            imuObs->sensorLabel = params_.imu_sensor_label;
            imuObs->sensorPose  = resolvedImuPose_;
            imuObs->timestamp   = ousterTsToMrpt(imuTs(i));

            imuObs->set(mrpt::obs::IMU_X_ACC, static_cast<double>(imuAcc(i, 0)));
            imuObs->set(mrpt::obs::IMU_Y_ACC, static_cast<double>(imuAcc(i, 1)));
            imuObs->set(mrpt::obs::IMU_Z_ACC, static_cast<double>(imuAcc(i, 2)));
            imuObs->set(mrpt::obs::IMU_WX, static_cast<double>(imuGyro(i, 0)));
            imuObs->set(mrpt::obs::IMU_WY, static_cast<double>(imuGyro(i, 1)));
            imuObs->set(mrpt::obs::IMU_WZ, static_cast<double>(imuGyro(i, 2)));

            sendObservationsToFrontEnds(imuObs);
          }
        }

        auto obs = scanToObservation(*scan);
        if (obs && obs->pointcloud && obs->pointcloud->size() > 0)
        {
          paceReplay(obs->timestamp);
          sendObservationsToFrontEnds(obs);
          gotScan = true;
        }
      }
    }

    if (it == end)
    {
      MRPT_LOG_INFO("End of OSF file reached.");
      onDatasetPlaybackEnds();
      return;
    }
  }
}

// ============================================================================
// paceReplay: Enforce time_warp_scale pacing for PCAP replay
// ============================================================================
void OusterDirectInput::paceReplay(const mrpt::Clock::time_point& obsTimestamp)
{
  const double datasetTime = mrpt::Clock::toDouble(obsTimestamp);

  if (pcapLastWallclock_.has_value() && params_.time_warp_scale > 0)
  {
    const double dtDataset = datasetTime - pcapLastDatasetTime_;
    const double dtWall    = dtDataset / params_.time_warp_scale;

    const auto   now     = mrpt::Clock::now();
    const double elapsed = mrpt::system::timeDifference(pcapLastWallclock_.value(), now);

    if (elapsed < dtWall)
    {
      const double sleepSec = dtWall - elapsed;
      std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(sleepSec * 1e6)));
    }
  }

  pcapLastWallclock_   = mrpt::Clock::now();
  pcapLastDatasetTime_ = datasetTime;
}

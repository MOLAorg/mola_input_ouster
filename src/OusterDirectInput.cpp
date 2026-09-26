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
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/filesystem.h>

// Ouster SDK (0.16+):
#include <ouster/cartesian.h>
#include <ouster/client.h>
#include <ouster/image_processing.h>
#include <ouster/indexed_pcap_reader.h>
#include <ouster/lidar_scan.h>
#include <ouster/osf/meta_lidar_sensor.h>
#include <ouster/osf/meta_streaming_info.h>
#include <ouster/osf/reader.h>
#include <ouster/osf/stream_lidar_scan.h>
#include <ouster/pcap.h>
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

  // PCAP replay state (null unless in PCAP mode)
  std::unique_ptr<spc::PcapReader> pcapReader;
  int                              pcapLidarPort = 0;
  int                              pcapImuPort   = 0;
  uint64_t                         pcapStartUs   = 0;  // capture time of the first packet
  std::optional<uint64_t>          pcapFrameStartUs;  // capture time of the scan being batched

  // OSF replay state (null unless in OSF mode)
  std::unique_ptr<sosf::Reader>                osfReader;
  std::unique_ptr<sosf::MessagesStreamingIter> osfIter;
  std::unique_ptr<sosf::MessagesStreamingIter> osfEnd;
  uint32_t                                     osfStreamId  = 0;  // replayed LidarScan stream
  uint64_t                                     osfScanCount = 0;
  sosf::ts_t                                   osfStartTs{0};
  sosf::ts_t                                   osfEndTs{0};

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
  pcapIndexAbort_ = true;
  if (pcapIndexThread_.joinable())
  {
    pcapIndexThread_.join();
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

  pcapIndexAbort_ = true;
  if (pcapIndexThread_.joinable())
  {
    pcapIndexThread_.join();
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
  if (cfg.has("start_paused"))
  {
    params_.start_paused = cfg["start_paused"].as<bool>();
  }
  if (cfg.has("decimate_columns"))
  {
    params_.decimate_columns = cfg["decimate_columns"].as<int>();
  }
  if (cfg.has("decimate_rows"))
  {
    params_.decimate_rows = cfg["decimate_rows"].as<int>();
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

  ASSERTMSG_(
      params_.decimate_columns >= 1 && params_.decimate_rows >= 1,
      "'decimate_columns' and 'decimate_rows' must be >= 1.");

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

  {
    auto lck       = mrpt::lockHelper(dataset_ui_mtx_);
    timeWarpScale_ = params_.time_warp_scale;
    paused_        = params_.start_paused;
  }

  // ---- Start receiver thread for live mode ----
  if (isLiveMode())
  {
    receiverRunning_ = true;
    receiverThread_  = std::thread(&OusterDirectInput::receiverThreadFunc, this);
  }
  else if (!isOsfMode())
  {
    pcapIndexThread_ = std::thread(&OusterDirectInput::pcapIndexThreadFunc, this);
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
  ousterState_->pcapReader = std::make_unique<spc::PcapReader>(params_.pcap_file);

  // The first packet capture time is the origin of the playback time:
  if (ousterState_->pcapReader->next_packet() != 0)
  {
    ousterState_->pcapStartUs =
        static_cast<uint64_t>(ousterState_->pcapReader->current_info().timestamp.count());
  }
  ousterState_->pcapReader->reset();

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

  auto& reader = *(ousterState_->osfReader);

  // Replay one LidarScan stream (the first one, if the file has several):
  const auto scanStreams = reader.meta_store().find<sosf::LidarScanStreamMeta>();
  ASSERTMSG_(!scanStreams.empty(), "OSF file contains no LidarScan stream.");
  ousterState_->osfStreamId = scanStreams.begin()->first;
  if (scanStreams.size() > 1)
  {
    MRPT_LOG_WARN_FMT(
        "OSF file has %zu LidarScan streams, only the first one is replayed.", scanStreams.size());
  }

  // Number of scans and time span, for the playback UI:
  ousterState_->osfStartTs = reader.start_ts();
  ousterState_->osfEndTs   = reader.end_ts();
  if (const auto streamingInfo = reader.meta_store().get<sosf::StreamingInfo>(); streamingInfo)
  {
    const auto& stats = streamingInfo->stream_stats();
    if (const auto it = stats.find(ousterState_->osfStreamId); it != stats.end())
    {
      ousterState_->osfScanCount = it->second.message_count;
      ousterState_->osfStartTs   = it->second.start_ts;
      ousterState_->osfEndTs     = it->second.end_ts;
    }
  }
  MRPT_LOG_INFO_FMT(
      "OSF LidarScan stream: %lu scans, %.1f s.",
      static_cast<unsigned long>(ousterState_->osfScanCount),
      1e-9 * static_cast<double>((ousterState_->osfEndTs - ousterState_->osfStartTs).count()));

  osfStartReadingAt(reader.start_ts());
}

void OusterDirectInput::osfStartReadingAt(const sosf::ts_t& startTs)
{
  auto& reader          = *(ousterState_->osfReader);
  auto  range           = reader.messages({ousterState_->osfStreamId}, startTs, reader.end_ts());
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

  // Allocate a LidarScan. Built from the sensor info so it also gets the
  // number of columns per packet, which ScanBatcher checks on every packet.
  ousterState_->scan = std::make_unique<sc::LidarScan>(info);

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
  // Its data are expressed in that frame. This is consistent with how
  // mrpt::ros2bridge and BridgeROS2 work: they query /tf for
  // base_link → frame_id and set that as sensorPose.
  //
  // The Ouster sensor stores factory-calibrated 4×4 transforms (mm):
  //   lidar_to_sensor_transform: Lidar frame → Sensor housing frame
  //   imu_to_sensor_transform:   IMU frame   → Sensor housing frame
  //
  // The user provides sensor_mounting_pose = pose of the sensor housing
  // on the vehicle (base_link → os_sensor).
  //
  //   resolved_lidar_pose = mounting = base_link → os_sensor
  //     (the XYZ LUT already maps ranges into os_sensor, see scanToObservation)
  //   resolved_imu_pose   = mounting ∘ imu_to_sensor
  //                       = base_link → os_sensor → os_imu

  if (params_.lidar_sensor_pose_override.has_value())
  {
    resolvedLidarPose_ = params_.lidar_sensor_pose_override.value();
    MRPT_LOG_INFO_STREAM(
        "Using manual lidar_sensor_pose override: " << resolvedLidarPose_.asString());
  }
  else
  {
    // The XYZ LUT already applies lidar_to_sensor_transform, so points are
    // in os_sensor and must not be transformed by it again.
    resolvedLidarPose_ = params_.sensor_mounting_pose;
    MRPT_LOG_INFO_STREAM(
        "Lidar points frame pose on vehicle (base_link -> os_sensor): "
        << resolvedLidarPose_.asString() << "\n  (os_lidar->os_sensor, applied by the LUT: "
        << mat4dToPose(info.lidar_to_sensor_transform).asString() << ")");
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
// Point coordinates are in the Ouster *Sensor Coordinate Frame*: the LUT
// from make_xyz_lut(info, false) bakes lidar_to_sensor_transform in, which is
// the frame used by ouster-ros for /ouster/points. Hence, sensorPose is
// resolvedLidarPose_ = base_link -> os_sensor.
//
// Only every `decimate_columns`-th column and `decimate_rows`-th row are kept.
// ============================================================================
namespace
{
// Convert a float16 bit pattern to float32.
float f16_to_f32(uint16_t h)
{
  const uint32_t s = (h >> 15u) & 1u;
  const uint32_t e = (h >> 10u) & 0x1fu;
  const uint32_t m = h & 0x3ffu;
  uint32_t       bits;
  if (e == 0)
  {
    bits = (s << 31u) | ((m == 0) ? 0u : ((113u << 23u) | (m << 13u)));
  }
  else if (e == 31u)
  {
    bits = (s << 31u) | 0x7f800000u | (m << 13u);
  }
  else
  {
    bits = (s << 31u) | ((e + 112u) << 23u) | (m << 13u);
  }
  float f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

// Read access to a scalar channel field (UINT8/16/32 or FLOAT16) as float,
// resolved once per scan: looking a field up by name per pixel dominated the
// conversion cost.
class ScalarFieldReader
{
 public:
  ScalarFieldReader(const sc::LidarScan& scan, const std::string& name)
  {
    if (!scan.has_field(name))
    {
      return;
    }
    const auto& f = scan.field(name);
    tag_          = f.tag();
    data_         = f.get();
  }

  bool valid() const { return data_ != nullptr; }

  float operator[](std::size_t idx) const
  {
    switch (tag_)
    {
      case sc::ChanFieldType::UINT8:
        return static_cast<float>(static_cast<const uint8_t*>(data_)[idx]);
      case sc::ChanFieldType::UINT16:
        return static_cast<float>(static_cast<const uint16_t*>(data_)[idx]);
      case sc::ChanFieldType::UINT32:
        return static_cast<float>(static_cast<const uint32_t*>(data_)[idx]);
      case sc::ChanFieldType::FLOAT16:
        return f16_to_f32(static_cast<const sc::float16_t*>(data_)[idx].data);
      default:
        return 0.f;
    }
  }

 private:
  const void*       data_ = nullptr;
  sc::ChanFieldType tag_  = sc::ChanFieldType::VOID;
};
}  // namespace

mrpt::obs::CObservationPointCloud::Ptr OusterDirectInput::scanToObservation(
    const sc::LidarScan& scan)
{
  const ProfilerEntry tleg(profiler_, "scanToObservation");

  const auto W  = static_cast<std::size_t>(ousterState_->w);
  const auto H  = static_cast<std::size_t>(ousterState_->h);
  const auto dc = static_cast<std::size_t>(params_.decimate_columns);
  const auto dr = static_cast<std::size_t>(params_.decimate_rows);

  // Range image, row-major (H, W): pixel (row, col) -> flat index row * W + col.
  // The same flat index addresses the LUT rows and every other channel field.
  const auto&     rangeImg = scan.field<uint32_t>(sc::ChanField::RANGE);
  const uint32_t* range    = rangeImg.data();

  const ScalarFieldReader signal(scan, sc::ChanField::SIGNAL);
  const ScalarFieldReader reflectivity(scan, sc::ChanField::REFLECTIVITY);
  const bool              hasRGB = scan.has_field(sc::ChanField::RGB);

  // RGB raw bytes, only valid when hasRGB; layout [H, W, 3] row-major.
  // SDK v0.16.2 introduced FLOAT16 RGB for the new RNG19_RFL8_SIG16_NIR16_RGB16
  // profiles. Those values are raw HDR (linear, unbounded); we apply
  // AutoExposure (percentile contrast-stretch -> [0,1]) before scaling to uint8.
  // Older profiles use UINT8 packed bytes.
  const uint8_t*       rgbRaw = nullptr;
  std::vector<uint8_t> rgbBuf;
  if (hasRGB)
  {
    const auto& rgbField = scan.field(sc::ChanField::RGB);
    if (rgbField.tag() == sc::ChanFieldType::FLOAT16)
    {
      const auto*        f16ptr = rgbField.get<sc::float16_t>();
      const auto         nElems = static_cast<Eigen::Index>(H * W * 3);
      std::vector<float> floatBuf(static_cast<std::size_t>(nElems));

      // TensorMap shapes: [H, W, 3] row-major (RowMajor Tensor).
      using F16TensorMap = Eigen::TensorMap<const sc::rgb_img_t<sc::float16_t>>;
      using F32TensorMap = Eigen::TensorMap<sc::rgb_img_t<float>>;
      const auto   eH    = static_cast<Eigen::Index>(H);
      const auto   eW    = static_cast<Eigen::Index>(W);
      F16TensorMap inputMap(f16ptr, eH, eW, 3);
      F32TensorMap outputMap(floatBuf.data(), eH, eW, 3);

      ousterState_->rgbAutoExposure.update(inputMap, outputMap);

      rgbBuf.resize(static_cast<std::size_t>(nElems));
      for (std::size_t i = 0; i < rgbBuf.size(); ++i)
      {
        rgbBuf[i] = static_cast<uint8_t>(std::clamp(floatBuf[i] * 255.f, 0.f, 255.f));
      }
      rgbRaw = rgbBuf.data();
    }
    else
    {
      rgbRaw = rgbField.get<uint8_t>();
    }
  }

  // Per-column timestamps: find the first non-zero one to use as scan origin.
  const auto& timestamps = scan.timestamp();
  uint64_t    firstTs    = 0;
  for (Eigen::Index c = 0; c < timestamps.size(); ++c)
  {
    if (timestamps(c) != 0)
    {
      firstTs = static_cast<uint64_t>(timestamps(c));
      break;
    }
  }

  // First pass: count valid (non-zero range) pixels, so the cloud and all
  // its fields are allocated once and then filled through raw pointers.
  std::size_t nPts = 0;
  for (std::size_t row = 0; row < H; row += dr)
  {
    for (std::size_t col = 0; col < W; col += dc)
    {
      if (range[row * W + col] != 0)
      {
        nPts++;
      }
    }
  }

  auto pts = mrpt::maps::CGenericPointsMap::Create();

  if (signal.valid())
  {
    pts->registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY);
  }
  if (reflectivity.valid())
  {
    pts->registerField_float("reflectivity");
  }
  if (hasRGB)
  {
    pts->registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8);
    pts->registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gu8);
    pts->registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bu8);
  }
  pts->registerField_float("t");

  pts->resize(nPts);

  float* outT = pts->getPointsBufferRef_float_field("t")->data();
  float* outI =
      signal.valid()
          ? pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY)
                ->data()
          : nullptr;
  float* outRefl =
      reflectivity.valid() ? pts->getPointsBufferRef_float_field("reflectivity")->data() : nullptr;
  uint8_t* outR = nullptr;
  uint8_t* outG = nullptr;
  uint8_t* outB = nullptr;
  if (rgbRaw)
  {
    outR =
        pts->getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8)->data();
    outG =
        pts->getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gu8)->data();
    outB =
        pts->getPointsBufferRef_uint8_field(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bu8)->data();
  }

  // XYZ = range * direction + offset, as in the SDK's cartesian():
  const double* dir = ousterState_->xyzLut.direction.data();
  const double* ofs = ousterState_->xyzLut.offset.data();

  std::size_t i = 0;
  for (std::size_t col = 0; col < W; col += dc)
  {
    const uint64_t colTs = static_cast<uint64_t>(timestamps(static_cast<Eigen::Index>(col)));
    const float    tSecs = (colTs >= firstTs) ? static_cast<float>((colTs - firstTs) * 1e-9) : 0.f;

    for (std::size_t row = 0; row < H; row += dr)
    {
      const std::size_t idx = row * W + col;
      const auto        r   = static_cast<double>(range[idx]);
      if (r == 0)
      {
        continue;  // invalid point
      }

      const std::size_t k = idx * 3;
      pts->setPointFast(
          i, static_cast<float>(r * dir[k + 0] + ofs[k + 0]),
          static_cast<float>(r * dir[k + 1] + ofs[k + 1]),
          static_cast<float>(r * dir[k + 2] + ofs[k + 2]));

      outT[i] = tSecs;
      if (outI)
      {
        outI[i] = signal[idx];
      }
      if (outRefl)
      {
        outRefl[i] = reflectivity[idx];
      }
      if (outR)
      {
        const uint8_t* px = rgbRaw + idx * 3;
        outR[i]           = px[0];
        outG[i]           = px[1];
        outB[i]           = px[2];
      }
      i++;
    }
  }
  pts->mark_as_modified();

  auto obs         = mrpt::obs::CObservationPointCloud::Create();
  obs->pointcloud  = std::move(pts);
  obs->sensorLabel = params_.lidar_sensor_label;
  obs->sensorPose  = resolvedLidarPose_;
  // Scan-level timestamp: the first column timestamp (same origin as "t" fields).
  obs->timestamp = ousterTsToMrpt(firstTs);

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
  else
  {
    replaySpinOnce();
  }
}

// ============================================================================
// replaySpinOnce: Drive PCAP / OSF replay at the configured pace
//
// Never blocks: the replay time advances with the wall clock (times the
// playback speed), and every scan whose time has come is published. The
// next scan is decoded ahead and kept in pendingFrame_ until then.
// ============================================================================
void OusterDirectInput::replaySpinOnce()
{
  auto         lckUI         = mrpt::lockHelper(dataset_ui_mtx_);
  const double timeWarpScale = timeWarpScale_;
  const bool   paused        = paused_;
  const auto   teleportHere  = teleportHere_;
  teleportHere_.reset();
  lckUI.unlock();

  const auto   tNow    = mrpt::Clock::now();
  const double dtWall  = lastReplayWallclock_.has_value()
                             ? mrpt::system::timeDifference(*lastReplayWallclock_, tNow)
                             : 0.0;
  lastReplayWallclock_ = tNow;

  if (teleportHere.has_value() && *teleportHere < datasetUI_size())
  {
    // The scan at the new position is published right away, even if paused:
    replaySeek(*teleportHere);
  }
  else if (paused)
  {
    return;
  }

  if (!pendingFrame_)
  {
    pendingFrame_ = isOsfMode() ? readNextOsfFrame() : readNextPcapFrame();
  }
  if (!pendingFrame_)
  {
    MRPT_LOG_THROTTLE_INFO(10.0, "End of replayed file reached.");
    onDatasetPlaybackEnds();
    return;
  }

  // Replay as fast as possible: one scan per call.
  if (timeWarpScale <= 0)
  {
    publishFrame(*pendingFrame_);
    pendingFrame_.reset();
    replayTime_.reset();  // re-anchor if the speed becomes positive again
    return;
  }

  if (!replayTime_.has_value())
  {
    replayTime_ = pendingFrame_->t;  // (re)start: publish the first scan now
  }
  else if (!teleportHere.has_value())
  {
    *replayTime_ += dtWall * timeWarpScale;
  }

  // Bounded, so a replay slower than the requested speed does not build up an
  // ever-growing backlog (and stays responsive to pause and seek):
  constexpr size_t MAX_SCANS_PER_CALL = 5;
  size_t           nPublished         = 0;
  while (pendingFrame_ && pendingFrame_->t <= *replayTime_ && !requestedShutdown())
  {
    if (nPublished++ == MAX_SCANS_PER_CALL)
    {
      *replayTime_ = pendingFrame_->t;  // drop the lag
      break;
    }
    publishFrame(*pendingFrame_);
    pendingFrame_ = isOsfMode() ? readNextOsfFrame() : readNextPcapFrame();
  }
}

void OusterDirectInput::publishFrame(const ReplayFrame& frame)
{
  for (const auto& o : frame.obs)
  {
    sendObservationsToFrontEnds(o);
  }

  auto lck            = mrpt::lockHelper(dataset_ui_mtx_);
  lastPublishedIndex_ = frame.index;
  lastPublishedTime_  = frame.uiTime;
}

// ============================================================================
// readNextPcapFrame: Read packets until one full LiDAR scan is assembled.
// Packets are distinguished by destination UDP port (lidar vs imu).
// Returns nullopt at the end of the file.
// ============================================================================
std::optional<OusterDirectInput::ReplayFrame> OusterDirectInput::readNextPcapFrame()
{
  if (!ousterState_ || !ousterState_->pcapReader)
  {
    return {};
  }

  auto& reader = *(ousterState_->pcapReader);
  auto& scan   = *(ousterState_->scan);

  ReplayFrame frame;
  while (!requestedShutdown() && reader.next_packet() != 0)
  {
    const auto& pktInfo = reader.current_info();
    const auto  size    = pktInfo.payload_size;

    if (pktInfo.dst_port == ousterState_->pcapLidarPort &&
        size <= ousterState_->lidarPkt.buf.size())
    {
      const auto captureUs = static_cast<uint64_t>(pktInfo.timestamp.count());
      if (!ousterState_->pcapFrameStartUs)
      {
        ousterState_->pcapFrameStartUs = captureUs;
      }

      std::memcpy(ousterState_->lidarPkt.buf.data(), reader.current_data(), size);
      if (!(*ousterState_->batcher)(ousterState_->lidarPkt, scan))
      {
        continue;
      }
      // A scan is complete when the first packet of the next one arrives:
      const uint64_t scanStartUs     = *ousterState_->pcapFrameStartUs;
      ousterState_->pcapFrameStartUs = captureUs;

      const size_t index = nextFrameIndex_++;

      auto obs = scanToObservation(scan);
      if (obs && obs->pointcloud && obs->pointcloud->size() > 0)
      {
        frame.t      = mrpt::Clock::toDouble(obs->timestamp);
        frame.uiTime = 1e-6 * static_cast<double>(scanStartUs - ousterState_->pcapStartUs);
        frame.index  = index;
        frame.obs.push_back(obs);
        return frame;
      }
    }
    else if (
        pktInfo.dst_port == ousterState_->pcapImuPort && size <= ousterState_->imuPkt.buf.size())
    {
      std::memcpy(ousterState_->imuPkt.buf.data(), reader.current_data(), size);
      if (auto obs = imuToObservation(ousterState_->imuPkt.buf.data()); obs)
      {
        frame.obs.push_back(obs);
      }
    }
  }
  return {};
}

// ============================================================================
// readNextOsfFrame: Advance through the OSF LidarScan stream until one scan
// is decoded. Returns nullopt at the end of the file.
// ============================================================================
std::optional<OusterDirectInput::ReplayFrame> OusterDirectInput::readNextOsfFrame()
{
  if (!ousterState_ || !ousterState_->osfIter || !ousterState_->osfEnd)
  {
    return {};
  }

  auto& it  = *(ousterState_->osfIter);
  auto& end = *(ousterState_->osfEnd);

  ReplayFrame frame;
  while (it != end && !requestedShutdown())
  {
    const auto msg = *it;
    ++it;

    if (!msg.is<sosf::LidarScanStream>())
    {
      continue;
    }
    const size_t index = nextFrameIndex_++;

    auto scan = msg.decode_msg<sosf::LidarScanStream>();
    if (!scan)
    {
      continue;
    }

    // Emit IMU observations from embedded IMU fields (ACCEL32_GYRO32_NMEA
    // profile stores multiple IMU samples per LidarScan).
    // IMU_ACC is in m/s² and IMU_GYRO is in rad/s (non-legacy profile).
    if (scan->has_field(sc::ChanField::IMU_TIMESTAMP) && scan->has_field(sc::ChanField::IMU_ACC) &&
        scan->has_field(sc::ChanField::IMU_GYRO))
    {
      const sc::ArrayView1<uint64_t> imuTs   = scan->field(sc::ChanField::IMU_TIMESTAMP);
      const sc::ArrayView2<float>    imuAcc  = scan->field(sc::ChanField::IMU_ACC);
      const sc::ArrayView2<float>    imuGyro = scan->field(sc::ChanField::IMU_GYRO);

      for (std::size_t i = 0; i < imuTs.shape[0]; ++i)
      {
        if (imuTs(i) == 0)
        {
          continue;  // slot not filled (e.g. a partial first scan)
        }
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

        frame.obs.push_back(imuObs);
      }
    }

    auto obs = scanToObservation(*scan);
    if (obs && obs->pointcloud && obs->pointcloud->size() > 0)
    {
      frame.t      = mrpt::Clock::toDouble(obs->timestamp);
      frame.uiTime = 1e-9 * static_cast<double>((msg.ts() - ousterState_->osfStartTs).count());
      frame.index  = index;
      frame.obs.push_back(obs);
      return frame;
    }
  }
  return {};
}

// ============================================================================
// replaySeek: Continue replaying from the given scan number
// ============================================================================
void OusterDirectInput::replaySeek(size_t frameIndex)
{
  if (isOsfMode())
  {
    auto& reader = *(ousterState_->osfReader);

    std::optional<sosf::ts_t> ts;
    if (reader.has_message_idx())
    {
      const auto t =
          reader.ts_by_message_idx(ousterState_->osfStreamId, static_cast<uint32_t>(frameIndex));
      if (t.has_value())
      {
        ts = *t;
      }
    }
    if (!ts.has_value())
    {
      // Without a per-message index, assume evenly spaced scans:
      const auto   N    = std::max<uint64_t>(2, ousterState_->osfScanCount);
      const double frac = static_cast<double>(frameIndex) / static_cast<double>(N - 1);
      const auto   span = ousterState_->osfEndTs - ousterState_->osfStartTs;
      ts                = ousterState_->osfStartTs +
           sosf::ts_t(static_cast<int64_t>(frac * static_cast<double>(span.count())));
    }
    osfStartReadingAt(*ts);
  }
  else
  {
    ousterState_->pcapReader->seek(pcapFrameOffsets_.at(frameIndex));
    ousterState_->pcapFrameStartUs.reset();
  }

  // Drop any partially assembled scan from the previous position:
  ousterState_->batcher = std::make_unique<sc::ScanBatcher>(ousterState_->info);
  ousterState_->scan    = std::make_unique<sc::LidarScan>(ousterState_->info);

  nextFrameIndex_ = frameIndex;
  pendingFrame_.reset();
  replayTime_.reset();
}

// ============================================================================
// pcapIndexThreadFunc: Find where each scan starts in the PCAP file, so the
// replay can jump to any of them. A full pass over the file, hence done in
// the background while the replay goes on.
// ============================================================================
void OusterDirectInput::pcapIndexThreadFunc()
{
  try
  {
    spc::IndexedPcapReader reader(
        params_.pcap_file, std::vector<sc::SensorInfo>{ousterState_->info});

    uint64_t lastUs = ousterState_->pcapStartUs;
    while (!pcapIndexAbort_ && !requestedShutdown() && reader.next_packet() != 0)
    {
      reader.update_index_for_current_packet();
      lastUs = static_cast<uint64_t>(reader.current_info().timestamp.count());
    }
    if (pcapIndexAbort_ || requestedShutdown())
    {
      return;
    }

    pcapFrameOffsets_ = reader.get_index().frame_indices.at(0);
    pcapTotalTime_    = 1e-6 * static_cast<double>(lastUs - ousterState_->pcapStartUs);
    pcapIndexReady_   = true;

    MRPT_LOG_INFO_FMT("PCAP indexed: %zu scans, %.1f s.", pcapFrameOffsets_.size(), pcapTotalTime_);
  }
  catch (const std::exception& e)
  {
    MRPT_LOG_WARN_STREAM(
        "Could not index the PCAP file, no playback UI will be available:\n"
        << mrpt::exception_to_str(e));
  }
}

// ============================================================================
// Dataset_UI
// ============================================================================
#if defined(MOLA_KERNEL_DATASET_UI_HAS_ENABLED)
bool OusterDirectInput::datasetUI_enabled() const
{
  // A live sensor has no position nor duration to control:
  if (isLiveMode())
  {
    return false;
  }
  return isOsfMode() || pcapIndexReady_;
}
#endif

size_t OusterDirectInput::datasetUI_size() const
{
  if (isOsfMode())
  {
    return ousterState_ ? static_cast<size_t>(ousterState_->osfScanCount) : 0;
  }
  return pcapIndexReady_ ? pcapFrameOffsets_.size() : 0;
}

size_t OusterDirectInput::datasetUI_lastQueriedTimestep() const
{
  auto lck = mrpt::lockHelper(dataset_ui_mtx_);
  return lastPublishedIndex_;
}

double OusterDirectInput::datasetUI_playback_speed() const
{
  auto lck = mrpt::lockHelper(dataset_ui_mtx_);
  return timeWarpScale_;
}

void OusterDirectInput::datasetUI_playback_speed(double speed)
{
  auto lck       = mrpt::lockHelper(dataset_ui_mtx_);
  timeWarpScale_ = speed;
}

bool OusterDirectInput::datasetUI_paused() const
{
  auto lck = mrpt::lockHelper(dataset_ui_mtx_);
  return paused_;
}

void OusterDirectInput::datasetUI_paused(bool paused)
{
  auto lck = mrpt::lockHelper(dataset_ui_mtx_);
  paused_  = paused;
}

void OusterDirectInput::datasetUI_teleport(size_t timestep)
{
  auto lck      = mrpt::lockHelper(dataset_ui_mtx_);
  teleportHere_ = timestep;
}

#if defined(MOLA_KERNEL_DATASET_UI_HAS_TIME)
std::optional<double> OusterDirectInput::datasetUI_time() const
{
  auto lck = mrpt::lockHelper(dataset_ui_mtx_);
  return lastPublishedTime_;
}

std::optional<double> OusterDirectInput::datasetUI_total_time() const
{
  if (isOsfMode() && ousterState_)
  {
    return 1e-9 * static_cast<double>((ousterState_->osfEndTs - ousterState_->osfStartTs).count());
  }
  if (!isLiveMode() && pcapIndexReady_)
  {
    return pcapTotalTime_;
  }
  return std::nullopt;
}
#endif

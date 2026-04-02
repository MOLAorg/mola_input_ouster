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
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/filesystem.h>

// Ouster SDK — client:
#include <ouster/client.h>
#include <ouster/lidar_scan.h>
#include <ouster/types.h>

// Ouster SDK — pcap (only needed for PCAP replay):
#include <ouster/os_pcap.h>

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <thread>

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
  // Ouster timestamps are nanoseconds since Unix epoch when using PTP,
  // or nanoseconds since sensor boot for internal clock.
  // Convert to mrpt::Clock (which wraps std::chrono::system_clock).
  using namespace std::chrono;
  const auto dur = nanoseconds(nsec);
  const auto tp  = std::chrono::time_point<std::chrono::system_clock, nanoseconds>(dur);
  return mrpt::Clock::time_point(duration_cast<mrpt::Clock::duration>(tp.time_since_epoch()));
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
  // Live sensor connection (null in pcap mode)
  std::shared_ptr<ouster::sensor::client> client;

  // Metadata (populated in both modes)
  ouster::sensor::sensor_info info;

  // Packet format (derived from sensor_info)
  std::unique_ptr<ouster::sensor::packet_format> pf;

  // Batching packets into full scans
  std::unique_ptr<ouster::ScanBatcher> batcher;
  std::unique_ptr<ouster::LidarScan>   scan;

  // Lookup table for converting range -> XYZ
  ouster::XYZLut xyzLut;

  // Raw packet buffers
  std::vector<uint8_t> lidarBuf;
  std::vector<uint8_t> imuBuf;

  // Scan dimensions
  int w = 0;  // columns per revolution
  int h = 0;  // pixels per column (channels)

  // PCAP replay handle (null in live mode)
  std::shared_ptr<ouster::sensor_utils::playback_handle> pcapHandle;
  int                                                    pcapLidarPort = 0;
  int                                                    pcapImuPort   = 0;

  // Reusable scratch buffer for skipping unknown packets
  std::vector<uint8_t> scratchBuf;
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
    ouster::sensor_utils::replay_uninitialize(*(ousterState_->pcapHandle));
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
      !params_.sensor_hostname.empty() || !params_.pcap_file.empty(),
      "Either 'sensor_hostname' (live) or 'pcap_file' (replay) must be "
      "provided.");

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

  // --- Create Ouster state ---
  ousterState_ = std::make_unique<OusterState>();

  if (isLiveMode())
  {
    initLiveMode();
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

  auto ld_mode = ouster::sensor::lidar_mode_of_string(params_.lidar_mode);
  auto ts_mode = ouster::sensor::timestamp_mode_of_string(params_.timestamp_mode);

  ASSERTMSG_(ld_mode.has_value(), "Invalid lidar_mode: '"s + params_.lidar_mode + "'"s);
  ASSERTMSG_(ts_mode.has_value(), "Invalid timestamp_mode: '"s + params_.timestamp_mode + "'"s);

  ousterState_->client = ouster::sensor::init_client(
      params_.sensor_hostname, params_.udp_dest, ld_mode.value(), ts_mode.value(),
      params_.lidar_port, params_.imu_port);

  ASSERTMSG_(
      ousterState_->client,
      "Failed to connect to Ouster sensor at '"s + params_.sensor_hostname + "'"s);

  // Retrieve metadata
  auto metadata_str  = ouster::sensor::get_metadata(*(ousterState_->client));
  ousterState_->info = ouster::sensor::parse_metadata(metadata_str);

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

  ousterState_->info = ouster::sensor::parse_metadata(metadata_str);

  MRPT_LOG_INFO_STREAM(
      "Loaded Ouster metadata. Product: " << ousterState_->info.prod_line
                                          << "  SN: " << ousterState_->info.sn);

  // Open pcap file for stepwise playback
  ousterState_->pcapHandle = ouster::sensor_utils::replay_initialize(params_.pcap_file);
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
// Setup batcher, XYZLut, buffers from sensor_info
// ============================================================================
void OusterDirectInput::setupOusterFromInfo()
{
  const auto& info = ousterState_->info;

  ousterState_->w = info.format.columns_per_frame;
  ousterState_->h = info.format.pixels_per_column;

  MRPT_LOG_INFO_FMT("Ouster scan format: %d x %d (cols x rows)", ousterState_->w, ousterState_->h);

  // Packet format
  ousterState_->pf =
      std::make_unique<ouster::sensor::packet_format>(ouster::sensor::get_format(info));

  // Scan batcher
  ousterState_->batcher = std::make_unique<ouster::ScanBatcher>(info);

  // Allocate a LidarScan
  ousterState_->scan = std::make_unique<ouster::LidarScan>(
      ousterState_->w, ousterState_->h, info.format.udp_profile_lidar);

  // XYZ lookup table
  ousterState_->xyzLut = ouster::make_xyz_lut(info);

  // Allocate packet buffers
  const auto& pf = *(ousterState_->pf);
  ousterState_->lidarBuf.resize(pf.lidar_packet_size + 1);
  ousterState_->imuBuf.resize(pf.imu_packet_size + 1);

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
    const ouster::LidarScan& scan)
{
  const ProfilerEntry tleg(profiler_, "scanToObservation");

  const int  W        = ousterState_->w;
  const int  H        = ousterState_->h;
  const auto totalPts = static_cast<std::size_t>(W) * static_cast<std::size_t>(H);

  // Convert range data to XYZ using the precomputed lookup table.
  // cartesianT() returns an Eigen::Array<double, -1, 3> of shape (W*H, 3).
  auto cloud = ouster::cartesianT(scan, ousterState_->xyzLut);

  // Get the range field to filter out invalid (zero-range) points.
  // range is a 2D Eigen array of shape (H, W).
  auto range = scan.field<uint32_t>(ouster::sensor::ChanField::RANGE);

  // Build MRPT point cloud
  auto pts = mrpt::maps::CSimplePointsMap::Create();
  pts->reserve(totalPts);

  for (std::size_t col = 0; col < static_cast<std::size_t>(W); ++col)
  {
    for (std::size_t row = 0; row < static_cast<std::size_t>(H); ++row)
    {
      if (range(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) == 0)
      {
        continue;  // invalid point
      }

      // cartesian() output is row-major (H, W):
      //   pixel at (row, col) -> flat index = row * W + col
      const auto idx = static_cast<Eigen::Index>(row * static_cast<std::size_t>(W) + col);
      const auto x   = static_cast<float>(cloud(idx, 0));
      const auto y   = static_cast<float>(cloud(idx, 1));
      const auto z   = static_cast<float>(cloud(idx, 2));
      pts->insertPointFast(x, y, z);
    }
  }

  // Determine scan timestamp from column timestamps.
  // Use the middle column's timestamp as the representative time.
  const auto& timestamps = scan.timestamp();
  const auto  midCol     = static_cast<Eigen::Index>(W / 2);
  uint64_t    scanTsNsec = 0;

  if (midCol < timestamps.size() && timestamps(midCol) != 0)
  {
    scanTsNsec = static_cast<uint64_t>(timestamps(midCol));
  }
  else
  {
    // Fallback: use first non-zero timestamp
    for (Eigen::Index c = 0; c < timestamps.size(); ++c)
    {
      if (timestamps(c) != 0)
      {
        scanTsNsec = static_cast<uint64_t>(timestamps(c));
        break;
      }
    }
  }

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
// Units are converted from the raw sensor format:
//   acceleration:      g      → m/s²
//   angular velocity:  deg/s  → rad/s
// ============================================================================
mrpt::obs::CObservationIMU::Ptr OusterDirectInput::imuToObservation(const uint8_t* buf)
{
  const ProfilerEntry tleg(profiler_, "imuToObservation");

  const auto& pf = *(ousterState_->pf);

  // Extract IMU fields from packet buffer using the packet_format API.
  //   imu_gyro_ts(buf)    -> uint64_t nanoseconds
  //   imu_la_{x,y,z}(buf) -> float   (acceleration in g)
  //   imu_av_{x,y,z}(buf) -> float   (angular velocity in deg/s)
  //
  // CObservationIMU expects m/s² and rad/s, so we convert here.
  // Reference: Ouster Sensor Docs, "IMU Data Format" section.
  constexpr double G_TO_MS2 = 9.80665;

  const uint64_t tsNsec = pf.imu_gyro_ts(buf);

  const auto laX = static_cast<double>(pf.imu_la_x(buf)) * G_TO_MS2;
  const auto laY = static_cast<double>(pf.imu_la_y(buf)) * G_TO_MS2;
  const auto laZ = static_cast<double>(pf.imu_la_z(buf)) * G_TO_MS2;
  const auto avX = mrpt::DEG2RAD(static_cast<double>(pf.imu_av_x(buf)));
  const auto avY = mrpt::DEG2RAD(static_cast<double>(pf.imu_av_y(buf)));
  const auto avZ = mrpt::DEG2RAD(static_cast<double>(pf.imu_av_z(buf)));

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

  auto&       cli     = *(ousterState_->client);
  const auto& pf      = *(ousterState_->pf);
  auto&       batcher = *(ousterState_->batcher);
  auto&       scan    = *(ousterState_->scan);

  while (receiverRunning_ && !requestedShutdown())
  {
    try
    {
      const auto st = ouster::sensor::poll_client(cli, /*timeout_sec=*/1);

      if (st & ouster::sensor::CLIENT_ERROR)
      {
        MRPT_LOG_ERROR("Ouster poll_client returned CLIENT_ERROR.");
        break;
      }
      if (st & ouster::sensor::EXIT)
      {
        MRPT_LOG_INFO("Ouster poll_client returned EXIT.");
        break;
      }

      // ---- LiDAR data ----
      if (st & ouster::sensor::LIDAR_DATA)
      {
        if (ouster::sensor::read_lidar_packet(cli, ousterState_->lidarBuf.data(), pf))
        {
          // Feed packet to batcher; returns true when a
          // full scan is assembled.
          if (batcher(ousterState_->lidarBuf.data(), scan))
          {
            auto obs = scanToObservation(scan);
            if (obs && obs->pointcloud && obs->pointcloud->size() > 0)
            {
              sendObservationsToFrontEnds(obs);
            }
          }
        }
      }

      // ---- IMU data ----
      if (st & ouster::sensor::IMU_DATA)
      {
        if (ouster::sensor::read_imu_packet(cli, ousterState_->imuBuf.data(), pf))
        {
          auto obs = imuToObservation(ousterState_->imuBuf.data());
          if (obs)
          {
            sendObservationsToFrontEnds(obs);
          }
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
    ouster::sensor_utils::packet_info pktInfo;
    if (!ouster::sensor_utils::next_packet_info(handle, pktInfo))
    {
      // End of file
      MRPT_LOG_INFO("End of PCAP file reached.");
      onDatasetPlaybackEnds();
      return;
    }

    const int  dstPort     = pktInfo.dst_port;
    const auto payloadSize = static_cast<std::size_t>(pktInfo.payload_size);

    if (dstPort == lidarPort && payloadSize <= ousterState_->lidarBuf.size())
    {
      // Read lidar packet
      const auto nRead = ouster::sensor_utils::read_packet(
          handle, ousterState_->lidarBuf.data(), ousterState_->lidarBuf.size());

      if (nRead > 0)
      {
        if (batcher(ousterState_->lidarBuf.data(), scan))
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
    else if (dstPort == imuPort && payloadSize <= ousterState_->imuBuf.size())
    {
      // Read IMU packet
      const auto nRead = ouster::sensor_utils::read_packet(
          handle, ousterState_->imuBuf.data(), ousterState_->imuBuf.size());

      if (nRead > 0)
      {
        auto obs = imuToObservation(ousterState_->imuBuf.data());
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
      ouster::sensor_utils::read_packet(
          handle, ousterState_->scratchBuf.data(), ousterState_->scratchBuf.size());
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

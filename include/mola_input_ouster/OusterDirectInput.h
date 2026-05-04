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
 * @file   OusterDirectInput.h
 * @brief  RawDataSource directly from an Ouster LiDAR via the Ouster SDK
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */
#pragma once

#include <mola_kernel/interfaces/RawDataSourceBase.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3D.h>

#include <atomic>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// Forward declarations to isolate Ouster SDK headers from downstream (SDK 0.16+):
namespace ouster::sdk::core
{
class LidarScan;
class SensorInfo;
class PacketFormat;
}  // namespace ouster::sdk::core

namespace ouster::sdk::sensor
{
class SensorPacketSource;
}  // namespace ouster::sdk::sensor

namespace ouster::sdk::osf
{
class Reader;
struct MessagesStreamingIter;
}  // namespace ouster::sdk::osf

namespace mola
{
/** RawDataSource from an Ouster LiDAR sensor using the native Ouster C++ SDK.
 *
 *  This module connects directly to an Ouster sensor (or replays a .pcap
 *  file) without requiring any ROS middleware. It produces
 *  mrpt::obs::CObservationPointCloud (LiDAR) and mrpt::obs::CObservationIMU
 *  observations and pushes them into the MOLA pipeline.
 *
 *  ## Coordinate frames and sensorPose
 *
 *  Point cloud coordinates are in the Ouster **Sensor Coordinate Frame**
 *  (the SDK's `make_xyz_lut(info)` bakes `lidar_to_sensor_transform` into
 *  the lookup table). IMU readings are in the **IMU frame**.
 *
 *  Each observation's `sensorPose` is set to the full transform from
 *  `base_link` to the respective sensor frame, matching the convention
 *  used by `mrpt::ros2bridge` and `mola::BridgeROS2`:
 *
 *    lidar sensorPose = sensor_mounting_pose (+) lidar_to_sensor_transform
 *    IMU   sensorPose = sensor_mounting_pose (+) imu_to_sensor_transform
 *
 *  Both intrinsic transforms are read automatically from the sensor
 *  firmware metadata (`sensor_info`). The user only needs to provide
 *  `sensor_mounting_pose` (pose of the Ouster housing on the vehicle).
 *
 *  ## Live mode
 *  Set `sensor_hostname` to connect to a live sensor.
 *
 *  ## PCAP replay mode
 *  Set `pcap_file` and `metadata_json` to replay a recorded capture.
 *
 *  ## OSF replay mode
 *  Set `osf_file` to replay an Ouster `.osf` recording. Sensor metadata and
 *  scan geometry are read directly from the OSF file; no separate metadata JSON
 *  is required.
 *
 *  ## YAML parameters
 *  ```yaml
 *  params:
 *    # --- Live mode (exclusive with pcap_file / osf_file) ---
 *    sensor_hostname: "os-122xxxxxxxxx.local"
 *    udp_dest: ""                   # empty = auto-detect
 *    lidar_port: 0                  # 0 = auto
 *    imu_port: 0                    # 0 = auto
 *
 *    # --- PCAP replay mode ---
 *    pcap_file: "/path/to/capture.pcap"
 *    metadata_json: "/path/to/metadata.json"
 *    time_warp_scale: 1.0
 *
 *    # --- OSF replay mode ---
 *    osf_file: "/path/to/recording.osf"
 *    time_warp_scale: 1.0
 *
 *    # --- Sensor configuration (live mode only) ---
 *    lidar_mode: "MODE_1024x10"
 *    timestamp_mode: "TIME_FROM_PTP_1588"
 *
 *    # --- Observation labeling ---
 *    lidar_sensor_label: "lidar"
 *    imu_sensor_label: "imu"
 *
 *    # --- Sensor housing pose on the vehicle (base_link → os_sensor) ---
 *    # The factory-calibrated lidar-to-sensor and imu-to-sensor
 *    # intrinsic transforms are read from the sensor metadata and
 *    # composed automatically:
 *    #   lidar sensorPose = mounting (+) lidar_to_sensor
 *    #   IMU   sensorPose = mounting (+) imu_to_sensor
 *    sensor_mounting_pose: "0 0 0 0 0 0"   # x y z yaw_deg pitch_deg roll_deg
 *
 *    # --- (Optional) Manual overrides for individual sensor poses ---
 *    # When set, these bypass the automatic composition and set the
 *    # observation sensorPose directly (base_link → sensor frame).
 *    # lidar_sensor_pose: "0 0 0 0 0 0"
 *    # imu_sensor_pose: "0 0 0 0 0 0"
 *  ```
 *
 * \ingroup mola_input_ouster_grp
 */
class OusterDirectInput : public RawDataSourceBase
{
  DEFINE_MRPT_OBJECT(OusterDirectInput, mola)

 public:
  OusterDirectInput();
  ~OusterDirectInput() override;

  // Forbid copy/move
  OusterDirectInput(const OusterDirectInput&)            = delete;
  OusterDirectInput& operator=(const OusterDirectInput&) = delete;
  OusterDirectInput(OusterDirectInput&&)                 = delete;
  OusterDirectInput& operator=(OusterDirectInput&&)      = delete;

  // See docs in base class
  void spinOnce() override;

  // See docs in base class
  void onQuit() override;

 protected:
  // See docs in base class
  void initialize_rds(const Yaml& cfg) override;

 private:
  // ---- Configuration ----
  struct Params
  {
    // Live mode
    std::string sensor_hostname;
    std::string udp_dest;
    int         lidar_port = 0;
    int         imu_port   = 0;

    // PCAP replay mode
    std::string pcap_file;
    std::string metadata_json;

    // OSF replay mode
    std::string osf_file;

    double time_warp_scale = 1.0;

    // Sensor configuration (live)
    std::string lidar_mode     = "MODE_1024x10";
    std::string timestamp_mode = "TIME_FROM_PTP_1588";

    // Observation labels
    std::string lidar_sensor_label = "lidar";
    std::string imu_sensor_label   = "imu";

    // Pose of the sensor housing on the vehicle
    mrpt::poses::CPose3D sensor_mounting_pose;

    // Optional manual overrides — when set, bypass auto-composition
    // with the factory-calibrated intrinsic transforms.
    std::optional<mrpt::poses::CPose3D> lidar_sensor_pose_override;
    std::optional<mrpt::poses::CPose3D> imu_sensor_pose_override;
  };
  Params params_;

  bool isLiveMode() const { return params_.pcap_file.empty() && params_.osf_file.empty(); }
  bool isOsfMode() const { return !params_.osf_file.empty(); }

  // Resolved sensorPose for observations:
  //  - resolvedLidarPose_: base_link → lidar frame (os_sensor ∘ lidar_to_sensor)
  //  - resolvedImuPose_:   base_link → IMU frame   (os_sensor ∘ imu_to_sensor)
  // Points stay in the lidar frame, IMU data stays in the IMU frame.
  mrpt::poses::CPose3D resolvedLidarPose_;
  mrpt::poses::CPose3D resolvedImuPose_;

  // ---- Ouster SDK state (opaque, defined in .cpp via PIMPL) ----
  struct OusterState;
  std::unique_ptr<OusterState> ousterState_;

  void setupOusterFromInfo();

  // ---- Initialization per mode ----
  void initLiveMode();
  void initPcapMode();
  void initOsfMode();

  // ---- Live receiver thread ----
  std::thread       receiverThread_;
  std::atomic<bool> receiverRunning_{false};
  void              receiverThreadFunc();

  // ---- PCAP replay state ----
  std::optional<mrpt::Clock::time_point> pcapLastWallclock_;
  double                                 pcapLastDatasetTime_ = 0;

  void pcapSpinOnce();
  void osfSpinOnce();
  void paceReplay(const mrpt::Clock::time_point& obsTimestamp);

  // ---- Conversions ----
  mrpt::obs::CObservationPointCloud::Ptr scanToObservation(
      const ouster::sdk::core::LidarScan& scan);

  mrpt::obs::CObservationIMU::Ptr imuToObservation(const uint8_t* buf);
};

}  // namespace mola

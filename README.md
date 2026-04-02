# mola_input_ouster
Provides a MOLA `RawDataSourceBase` module for
[Ouster](https://ouster.com/) LiDAR sensors using the native
[Ouster C++ SDK](https://github.com/ouster-lidar/ouster-sdk),
**without requiring any ROS middleware**.

This module can operate in two modes:

- **Live sensor**: Connects to an Ouster sensor on the network.
- **PCAP replay**: Replays a recorded `.pcap` capture file.

It produces `mrpt::obs::CObservationPointCloud` and
`mrpt::obs::CObservationIMU` observations for downstream consumption
by `mola::LidarOdometry`, state estimators, or any other
`mola::RawDataConsumer`.

## Build dependencies

- [mola_kernel](https://github.com/MOLAorg/mola/tree/develop/mola_kernel)
- [mola_yaml](https://github.com/MOLAorg/mola/tree/develop/mola_yaml)
- [mrpt](https://github.com/MRPT/mrpt) (mrpt-obs, mrpt-maps)
- [Ouster SDK](https://github.com/ouster-lidar/ouster-sdk) (ouster_client, ouster_pcap)

## Usage: Live LiDAR odometry

```bash
OUSTER_HOSTNAME=os-122xxxxxxxxx.local \
mola-cli -c $(mola-dir mola_input_ouster)/mola-cli-launchs/lidar_odometry_ouster_live.yaml
```

Or using the convenience script:

```bash
mola-lo-gui-ouster-live os-122xxxxxxxxx.local
```

For LiDAR-Inertial Odometry (LIO) with IMU deskewing:

```bash
MOLA_DESKEW_METHOD=MotionCompensationMethod::IMU \
mola-lo-gui-ouster-live os-122xxxxxxxxx.local
```

## Usage: PCAP replay

```bash
OUSTER_PCAP=/path/to/capture.pcap \
OUSTER_META=/path/to/metadata.json \
mola-cli -c $(mola-dir mola_input_ouster)/mola-cli-launchs/lidar_odometry_ouster_pcap.yaml
```

Or using the convenience script:

```bash
mola-lo-gui-ouster-pcap /path/to/capture.pcap /path/to/metadata.json
```

## YAML configuration

All parameters are documented in the class doxygen:
[`mola::OusterDirectInput`](include/mola_input_ouster/OusterDirectInput.h).

The most important ones:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sensor_hostname` | (required for live) | Ouster sensor hostname or IP |
| `pcap_file` | (required for replay) | Path to `.pcap` capture file |
| `metadata_json` | (required for replay) | Path to Ouster metadata JSON file |
| `lidar_mode` | `MODE_1024x10` | Ouster resolution/rate (live only) |
| `timestamp_mode` | `TIME_FROM_PTP_1588` | Ouster timestamp source (live only) |
| `lidar_sensor_label` | `lidar` | Sensor label for LiDAR observations |
| `imu_sensor_label` | `imu` | Sensor label for IMU observations |
| `sensor_mounting_pose` | `0 0 0 0 0 0` | Pose of sensor housing on vehicle (`base_link` → `os_sensor`), `x y z yaw_deg pitch_deg roll_deg` |
| `lidar_sensor_pose` | (auto) | Manual override: `base_link` → lidar frame (bypasses intrinsic composition) |
| `imu_sensor_pose` | (auto) | Manual override: `base_link` → IMU frame (bypasses intrinsic composition) |
| `time_warp_scale` | `1.0` | Replay speed multiplier (PCAP only) |

## Coordinate frames and `sensorPose`

In MRPT/MOLA, `CObservation::sensorPose` is the SE(3) pose of the
sensor's own coordinate frame w.r.t. the vehicle frame (`base_link`).
**Point coordinates inside `CObservationPointCloud::pointcloud` are
expressed in the lidar frame, and IMU readings in `CObservationIMU` are
expressed in the IMU frame.** The `sensorPose` tells downstream consumers
where each of those frames sits on the vehicle. This is consistent with
how `mrpt::ros2bridge` and `mola::BridgeROS2` handle observations.

## Sensor intrinsic transforms

Ouster sensors store factory-calibrated 4×4 homogeneous transforms
in their firmware metadata (queried via the sensor HTTP API, or read
from the metadata JSON in PCAP replay mode):

- **`lidar_to_sensor_transform`** — Lidar frame → sensor housing frame
  (`os_lidar` → `os_sensor`). Typically a 180° Z-rotation plus a Z
  offset (lidar aperture height above the housing origin).
- **`imu_to_sensor_transform`** — IMU frame → sensor housing frame
  (`os_imu` → `os_sensor`). Typically a small translation (the IMU
  chip position inside the housing).

Both are available in the SDK's `sensor_info` struct.

By default, this module **automatically composes** the user-provided
`sensor_mounting_pose` (pose of the housing on the vehicle, i.e.
`base_link` → `os_sensor`) with each factory intrinsic:

```
lidar sensorPose = sensor_mounting_pose (+) lidar_to_sensor_transform
                 = base_link → os_sensor → os_lidar

IMU   sensorPose = sensor_mounting_pose (+) imu_to_sensor_transform
                 = base_link → os_sensor → os_imu
```

This matches the TF tree published by the official ouster-ros driver.
Point data stays in the lidar frame; IMU data stays in the IMU frame.

If `lidar_sensor_pose` or `imu_sensor_pose` are set explicitly in YAML,
they **override** the automatic composition and are used directly as the
observation's `sensorPose` (still interpreted as `base_link` → sensor
frame).

## Environment variables

The mola-cli launch files support these environment variables,
following the same conventions as `mola_lidar_odometry`:

| Variable | Default | Description |
|----------|---------|-------------|
| `OUSTER_HOSTNAME` | (none) | Sensor hostname for live mode |
| `OUSTER_PCAP` | (none) | PCAP file path for replay mode |
| `OUSTER_META` | (none) | Metadata JSON file path |
| `OUSTER_LIDAR_MODE` | `MODE_1024x10` | LiDAR resolution/rate |
| `OUSTER_TIMESTAMP_MODE` | `TIME_FROM_PTP_1588` | Timestamp source |
| `MOLA_LIDAR_NAME` | `lidar` | Sensor label |
| `MOLA_IMU_NAME` | `imu` | IMU sensor label |
| `MOLA_TIME_WARP` | `1.0` | Replay speed |
| `SENSOR_POSE_{X,Y,Z,YAW,PITCH,ROLL}` | `0` | Sensor housing mounting extrinsics |

## Ouster SDK compatibility

This module targets Ouster SDK ≥0.11.0 (the version where `init_client`
uses `std::optional` for lidar/timestamp modes). Minor API adaptations
may be needed for older or newer SDK versions. See the notes at the top
of `OusterDirectInput.cpp`.

## License

GPL-3.0. See [LICENSE](LICENSE).

Part of the [MOLA](https://github.com/MOLAorg/mola) project.

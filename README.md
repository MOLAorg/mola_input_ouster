# mola_input_ouster
Provides a MOLA `RawDataSourceBase` module for
[Ouster](https://ouster.com/) LiDAR sensors using the native
[Ouster C++ SDK](https://github.com/ouster-lidar/ouster-sdk),
**without requiring any ROS middleware**.

This module can operate in three modes:

- **Live sensor**: Connects to an Ouster sensor on the network.
- **PCAP replay**: Replays a recorded `.pcap` capture file.
- **OSF replay**: Replays an Ouster `.osf` recording (sensor metadata is
  embedded in the file — no separate JSON required).

It produces `mrpt::obs::CObservationPointCloud` and
`mrpt::obs::CObservationIMU` observations for downstream consumption
by `mola::LidarOdometry`, state estimators, or any other
`mola::RawDataConsumer`.


| Distro | Build dev | Release |
| --- | --- | --- |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mola_input_ouster__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mola_input_ouster__ubuntu_jammy_amd64/) | [![Version](https://img.shields.io/ros/v/humble/mola_input_ouster)](https://index.ros.org/?search_packages=true&pkgs=mola_input_ouster) |
| ROS 2 Jazzy (u24.04) | [![Build Status](https://build.ros2.org/job/Jdev__mola_input_ouster__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mola_input_ouster__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/jazzy/mola_input_ouster)](https://index.ros.org/?search_packages=true&pkgs=mola_input_ouster) |
| ROS 2 Kilted (u24.04) | [![Build Status](https://build.ros2.org/job/Kdev__mola_input_ouster__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mola_input_ouster__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/kilted/mola_input_ouster)](https://index.ros.org/?search_packages=true&pkgs=mola_input_ouster) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mola_input_ouster__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mola_input_ouster__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/rolling/mola_input_ouster)](https://index.ros.org/?search_packages=true&pkgs=mola_input_ouster) |


## Usage: OSF replay (just view, no SLAM)

```bash
OUSTER_OSF=/path/to/recording.osf \
mola-cli  $(mola-dir mola_input_ouster)/mola-cli-launchs/osf_ouster_just_view.yaml
```

## Build dependencies

- [mola_kernel](https://github.com/MOLAorg/mola/tree/develop/mola_kernel)
- [mola_yaml](https://github.com/MOLAorg/mola/tree/develop/mola_yaml)
- [mrpt](https://github.com/MRPT/mrpt) (mrpt-obs, mrpt-maps)
- [Ouster SDK](https://github.com/ouster-lidar/ouster-sdk) (ouster_client, ouster_pcap,
  ouster_osf) — bundled as a git submodule (see below)

### Ouster SDK: bundled submodule vs. system installation

The Ouster SDK is included as a **git submodule** under `ouster-sdk/`, so the
package is self-contained and requires no separate SDK installation.

Clone with submodules:

```bash
git clone --recurse-submodules https://github.com/MOLAorg/mola_input_ouster.git
```

Or, if you already cloned without `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

If a system-wide (or user-installed) Ouster SDK is already present on your
machine, CMake will prefer it automatically. To force the bundled copy
regardless, pass `-DUSE_BUNDLED_OUSTER_SDK=ON` to CMake.

## Usage: Sensor check (just view, no SLAM)

Quickly verify connectivity and inspect the raw point clouds without running
any odometry:

```bash
OUSTER_HOSTNAME=os-122xxxxxxxxx.local \
mola-cli  $(mola-dir mola_input_ouster)/mola-cli-launchs/live_ouster_just_view.yaml
```

Same for a recorded PCAP file:

```bash
OUSTER_PCAP=/path/to/capture.pcap \
OUSTER_META=/path/to/metadata.json \
mola-cli  $(mola-dir mola_input_ouster)/mola-cli-launchs/pcap_ouster_just_view.yaml
```

## Usage: Live LiDAR odometry

```bash
OUSTER_HOSTNAME=os-122xxxxxxxxx.local \
mola-cli  $(mola-dir mola_input_ouster)/mola-cli-launchs/lidar_odometry_ouster_live.yaml
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

### Changing the lidar mode (resolution / spin rate)

The `OUSTER_LIDAR_MODE` environment variable sets the scan resolution and
rotation frequency. The format is `<columns>x<Hz>`:

| Value | Columns | Hz | Notes |
|---|---|---|---|
| `_512x10` | 512 | 10 | Fastest — lowest horizontal resolution |
| `_512x20` | 512 | 20 | |
| `_1024x10` | 1024 | 10 | **Default** |
| `_1024x20` | 1024 | 20 | Higher rate, same horizontal density |
| `_2048x10` | 2048 | 10 | Highest horizontal resolution |
| `_4096x5` | 4096 | 5 | Select sensors only |

Example — run at 2048 columns × 10 Hz:

```bash
OUSTER_HOSTNAME=os-122xxxxxxxxx.local \
OUSTER_LIDAR_MODE=_2048x10 \
mola-lo-gui-ouster-live os-122xxxxxxxxx.local
```

Or directly via the YAML `params` block:

```yaml
params:
  sensor_hostname: os-122xxxxxxxxx.local
  lidar_mode: _1024x20
  timestamp_mode: TIME_FROM_PTP_1588
```

## Usage: PCAP replay

```bash
OUSTER_PCAP=/path/to/capture.pcap \
OUSTER_META=/path/to/metadata.json \
mola-cli  $(mola-dir mola_input_ouster)/mola-cli-launchs/lidar_odometry_ouster_pcap.yaml
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
| `pcap_file` | (required for PCAP) | Path to `.pcap` capture file |
| `metadata_json` | (required for PCAP) | Path to Ouster metadata JSON file |
| `osf_file` | (required for OSF) | Path to `.osf` recording file |
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
| `OUSTER_OSF` | (none) | OSF file path for OSF replay mode |
| `OUSTER_LIDAR_MODE` | `MODE_1024x10` | LiDAR resolution/rate |
| `OUSTER_TIMESTAMP_MODE` | `TIME_FROM_PTP_1588` | Timestamp source |
| `MOLA_LIDAR_NAME` | `lidar` | Sensor label |
| `MOLA_IMU_NAME` | `imu` | IMU sensor label |
| `MOLA_TIME_WARP` | `1.0` | Replay speed |
| `SENSOR_POSE_{X,Y,Z,YAW,PITCH,ROLL}` | `0` | Sensor housing mounting extrinsics |

## Ouster SDK compatibility

The bundled submodule tracks Ouster SDK **v0.16.x** (currently pinned to
`v0.16.2`). This is the same SDK version used by the official
[ouster-ros](https://github.com/ouster-lidar/ouster-ros) driver.

The module requires SDK ≥0.11.0 (the version where `init_client` uses
`std::optional` for lidar/timestamp modes). See the notes at the top of
`OusterDirectInput.cpp` for any API adaptation details.

## License

GPL-3.0. See [LICENSE](LICENSE).

Part of the [MOLA](https://github.com/MOLAorg/mola) project.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_input_ouster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* ci: make scripts/release.py identical across repos by auto-discovering packages
* fix .gitignore: anchor build-* to directories only
* ci: add clang-format and colcon build workflows
* Merge pull request `#3 <https://github.com/MOLAorg/mola_input_ouster/issues/3>`_ from MOLAorg/feat/dataset-ui
  Add playback UI (pause, speed, seek) for PCAP and OSF replay
* Replay: bound the scans published per call, re-anchor after max speed
  - Publish at most 5 due scans per spinOnce() and drop the remaining lag.
  Otherwise, when decoding is slower than the requested speed, each call
  has more scans due than the last, and pause/seek stop responding.
  - Reset the replay time when playing as fast as possible, so switching
  back to a positive speed does not stall until the wall clock catches up.
* Add playback UI (pause, speed, seek) for PCAP and OSF replay
  Implement mola::Dataset_UI, enabled only when replaying a file (via
  datasetUI_enabled(), when available in mola_kernel), so the MOLA GUI shows
  the usual dataset playback panel. There is none in live mode.
  - Replay no longer sleeps inside spinOnce(): the replay time advances with
  the wall clock times the playback speed, and each scan is published once
  its time comes. Pause and speed changes take effect immediately.
  - OSF: scan count and duration come from the stream stats, and seeking
  uses the per-message index (or even spacing, if the file has none).
  - PCAP: read with PcapReader. A background thread indexes the file so the
  replay can seek, and the panel shows up once that finishes.
  - New start_paused parameter (MOLA_DATASET_START_PAUSED in the launch files).
  - Fix: allocate the LidarScan from the sensor info, so it has the right
  number of columns per packet. Profiles not using the default one made
  ScanBatcher throw on the first packet.
* ci: add GitHub release workflow on version tags
  Adds scripts/release.py (version-consistency check + changelog-based
  release notes) and .github/workflows/github-release.yml, which creates
  a GitHub Release whenever a M.m.P tag is pushed. Mirrors the pipeline
  already used in nanoflann_vendor.
* Merge pull request `#2 <https://github.com/MOLAorg/mola_input_ouster/issues/2>`_ from MOLAorg/perf/osf-rev8
  Fix LiDAR sensorPose, speed up scan conversion, add scan decimation
* Docs: LiDAR sensorPose is the mounting pose; fix transform direction in log
* Fix LiDAR sensorPose, speed up scan conversion, add scan decimation
  - Fix: point clouds had lidar_to_sensor_transform applied twice. The SDK's
  XYZ LUT already maps ranges into os_sensor, and sensorPose composed the
  same transform (typically a 180 deg yaw) again, so the cloud was rotated
  with respect to the IMU frame. sensorPose is now the mounting pose alone.
  On a sideways-mounted sensor with the IMU gravity prior on, a 62 s run
  drifted 9 m with the old pose vs 0.1 m with the fix.
  - scanToObservation() resolved channel fields by name for every pixel. It
  now resolves them once per scan, sizes the cloud once and fills its
  buffers through raw pointers: 124 -> 28 ms per 4096x256 scan, producing
  identical point data.
  - New decimate_columns / decimate_rows parameters (OUSTER_DECIMATE_COLUMNS /
  OUSTER_DECIMATE_ROWS in the launch files) keep every N-th column / row,
  for high-resolution sensors whose full scans are more than LiDAR
  odometry needs.
  - OSF replay: skip IMU samples whose slot was never filled (zero
  timestamp), as happens in a partial first scan.
* Merge pull request `#1 <https://github.com/MOLAorg/mola_input_ouster/issues/1>`_ from MOLAorg/feat/mrpt3
  Port to MRPT 3.x
* Merge remote-tracking branch 'origin/develop' into feat/mrpt3
* build: fall back to bundled Ouster SDK when a system OusterSDK export is incomplete
  Another package in the workspace can export a CMake package also named
  OusterSDK; if that export is missing its generated Targets.cmake file,
  find_package() aborted the whole configure instead of just reporting
  not-found. Check for that file before trusting the export.
* launch: default viz module to MolaVizImGui with per-app imgui_app_name
  Sets a unique imgui_app_name in each launch file so Dear ImGui's
  layout persistence stores a separate UI configuration per app.
* docs: add ROS 2 Lyrical badge row, update Rolling to Ubuntu 26.04 (resolute)
* Port to mrpt3: fix cmake targets, yaml API, and mrpt3 API changes
* Port to MRPT 3.x: rename packages and targets
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

0.1.0 (2026-05-20)
------------------
* Initial release: live Ouster sensor and PCAP replay support.
* Contributors: Jose Luis Blanco-Claraco


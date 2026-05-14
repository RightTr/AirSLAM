# AirSLAM_ROS2 Change Log

This file summarizes the major changes in this repository relative to the upstream official repository: `sair-lab/AirSLAM`.

Upstream reference used for comparison: `sair-lab/AirSLAM` `master` at `d126577`.

## High-Level Summary

This fork is not a small patch set. It turns the original ROS1/offline-oriented AirSLAM codebase into a ROS2-based, Jetson-oriented online visual odometry/VIO package with TensorRT deployment support.

The main additions are:

- ROS2 migration based on `ament_cmake` and `rclcpp`
- online stereo + IMU data ingestion from ROS2 topics
- Jetson / AGX Orin oriented TensorRT deployment workflow
- online visual odometry executable and launch files
- IMU/VIO integration work in the online pipeline
- additional camera and runtime configs for RealSense and other sensors

## Local Commit History Beyond Upstream

These local commits represent the main development steps on this fork:

1. `abf018c` ROS2 Init
2. `2a2b266` README Update
3. `5e0cd7d` ROS Subscriber Init
4. `e714fed` Add Online DataInput Thread
5. `daae727` DataInput Thread make success
6. `e322df0` Online test success
7. `899d92e` Config Polymorphism
8. `cdaf6ba` Add ImuData Input
9. `864259b` Adding IMU Data build success
10. `bac51a8` VO with Imu Test failed
11. `7fa3d72` Add RealSense Online mode
12. `9fce560` Run AirSLAM VIO success
13. `b4c7daa` Add IMU noise parameter

## Detailed Changes

### 1. ROS2 Port

The repository has been migrated from ROS1-style integration to ROS2:

- `CMakeLists.txt` now uses `ament_cmake` and ROS2 package dependencies.
- `package.xml` was rewritten for ROS2 package format and runtime/build dependencies.
- ROS publishers were ported from `ros_publisher` to `ros2_publisher`.
- A new ROS2 subscriber implementation was added for online sensor input.
- ROS2 launch files were added under `launch/visual_odometry` and `launch/visual_odometry_online`.
- A dedicated RViz2 configuration was added: `rviz/vo_ros2.rviz`.

Key files:

- [CMakeLists.txt](/home/right/airslam_ws/src/AirSLAM_ROS2/CMakeLists.txt)
- [package.xml](/home/right/airslam_ws/src/AirSLAM_ROS2/package.xml)
- [src/ros2_publisher.cc](/home/right/airslam_ws/src/AirSLAM_ROS2/src/ros2_publisher.cc)
- [src/ros2_subsriber.cc](/home/right/airslam_ws/src/AirSLAM_ROS2/src/ros2_subsriber.cc)
- [include/ros2_publisher.h](/home/right/airslam_ws/src/AirSLAM_ROS2/include/ros2_publisher.h)
- [include/ros2_subscriber.h](/home/right/airslam_ws/src/AirSLAM_ROS2/include/ros2_subscriber.h)

### 2. Online Visual Odometry Mode

A new online mode was added. This is one of the biggest changes compared with upstream.

What was added:

- a new executable: `visual_odometry_online`
- online ROS2 launch entries
- online-specific config files
- runtime parameter passing for config path, camera config path, model dir, and saving dir

The online entry point is:

- [demo/visual_odometry_online.cpp](/home/right/airslam_ws/src/AirSLAM_ROS2/demo/visual_odometry_online.cpp)

The launch/config entry points are:

- [launch/visual_odometry_online/vo_online.launch.py](/home/right/airslam_ws/src/AirSLAM_ROS2/launch/visual_odometry_online/vo_online.launch.py)
- [launch/visual_odometry_online/vo_realsense.launch.py](/home/right/airslam_ws/src/AirSLAM_ROS2/launch/visual_odometry_online/vo_realsense.launch.py)
- [configs/visual_odometry_online/vo_online.yaml](/home/right/airslam_ws/src/AirSLAM_ROS2/configs/visual_odometry_online/vo_online.yaml)
- [configs/visual_odometry_online/vo_realsense.yaml](/home/right/airslam_ws/src/AirSLAM_ROS2/configs/visual_odometry_online/vo_realsense.yaml)

### 3. Online ROS2 Sensor Input Pipeline

The original repository is centered around dataset-driven processing. This fork adds a live sensor pipeline.

Implemented pieces:

- stereo image subscription from ROS2 topics
- IMU subscription from ROS2 topics
- stereo association based on timestamp difference thresholds
- image buffering and IMU buffering
- extraction of IMU measurements between consecutive image timestamps
- conversion from ROS messages to internal `StereoFrame` / `ImuData`

This is implemented mainly in:

- [src/ros2_subsriber.cc](/home/right/airslam_ws/src/AirSLAM_ROS2/src/ros2_subsriber.cc)
- [include/buffer.h](/home/right/airslam_ws/src/AirSLAM_ROS2/include/buffer.h)
- [include/queue.h](/home/right/airslam_ws/src/AirSLAM_ROS2/include/queue.h)

The online subscriber config was also extended with:

- `left_topic`
- `right_topic`
- `imu_topic`
- image/IMU buffer sizes
- image timestamp association thresholds

This config support lives in:

- [include/read_configs.h](/home/right/airslam_ws/src/AirSLAM_ROS2/include/read_configs.h)

### 4. VIO / IMU Integration

This fork does more than just add ROS2 subscriptions. It also threads IMU data into the front-end pipeline.

Observed changes include:

- `InputData` now carries `batch_imu_data`
- online frame ingestion calls `PopImuDataBetween(last_time, frame_time, imu_measurements)`
- IMU preintegration is applied before frame tracking
- IMU-related configs and camera noise parameters were added/extended

Relevant files:

- [src/map_builder.cc](/home/right/airslam_ws/src/AirSLAM_ROS2/src/map_builder.cc)
- [include/imu.h](/home/right/airslam_ws/src/AirSLAM_ROS2/include/imu.h)
- [src/map.cc](/home/right/airslam_ws/src/AirSLAM_ROS2/src/map.cc)
- [src/g2o_optimization/g2o_optimization.cc](/home/right/airslam_ws/src/AirSLAM_ROS2/src/g2o_optimization/g2o_optimization.cc)
- [src/camera.cc](/home/right/airslam_ws/src/AirSLAM_ROS2/src/camera.cc)

This aligns with your remembered milestones:

- IMU data input was added
- VIO was brought up successfully
- IMU noise parameters were added into camera/config handling

### 5. Jetson / AGX Orin / TensorRT Adaptation

This is the other major customization track in the fork.

The repository is clearly adapted for NVIDIA deployment:

- TensorRT remains a first-class backend in the build
- `README.md` explicitly documents Jetson AGX Orin and JetPack 6.2 as the tested environment
- ONNX-to-TensorRT engine generation steps were documented
- multiple inference modules were updated around dynamic-shape TensorRT execution
- TensorRT helper code under `3rdparty/tensorrtbuffer` was modified locally

Evidence in the codebase:

- [README.md](/home/right/airslam_ws/src/AirSLAM_ROS2/README.md)
- [3rdparty/tensorrtbuffer/include/buffers.h](/home/right/airslam_ws/src/AirSLAM_ROS2/3rdparty/tensorrtbuffer/include/buffers.h)
- [3rdparty/tensorrtbuffer/include/sample_entrypoints.h](/home/right/airslam_ws/src/AirSLAM_ROS2/3rdparty/tensorrtbuffer/include/sample_entrypoints.h)
- [3rdparty/tensorrtbuffer/src/sample_options.cpp](/home/right/airslam_ws/src/AirSLAM_ROS2/3rdparty/tensorrtbuffer/src/sample_options.cpp)
- [3rdparty/tensorrtbuffer/src/sample_utils.cpp](/home/right/airslam_ws/src/AirSLAM_ROS2/3rdparty/tensorrtbuffer/src/sample_utils.cpp)

Inference modules with local TensorRT-oriented changes:

- [src/super_point.cpp](/home/right/airslam_ws/src/AirSLAM_ROS2/src/super_point.cpp)
- [src/light_glue.cpp](/home/right/airslam_ws/src/AirSLAM_ROS2/src/light_glue.cpp)
- [src/super_glue.cpp](/home/right/airslam_ws/src/AirSLAM_ROS2/src/super_glue.cpp)
- [src/plnet.cpp](/home/right/airslam_ws/src/AirSLAM_ROS2/src/plnet.cpp)

From the implementation, the TensorRT work includes:

- explicit dynamic-shape optimization profiles
- engine deserialization / rebuild fallback flow
- serialized engine caching to disk
- FP16-oriented inference setup
- DLA enable hooks in inference builders

So your memory is correct: this fork is not just “using TensorRT”, it contains actual deployment-side adaptation work aimed at Jetson-class hardware, including AGX Orin.

### 6. RealSense Online Mode

A dedicated online RealSense path was added:

- camera config: `configs/camera/realsense_d435i.yaml`
- online VO config: `configs/visual_odometry_online/vo_realsense.yaml`
- launch file: `launch/visual_odometry_online/vo_realsense.launch.py`

This is a concrete addition beyond the generic online mode and matches your note that RealSense support was added later.

### 7. Configuration Refactor / Polymorphism

The config system was extended rather than just patched:

- `VisualOdometryOnlineConfigs` support was introduced
- subscriber config and online-specific runtime fields were added
- model paths are resolved programmatically
- camera config path and saving dir are injected at runtime through ROS2 parameters

This is mainly visible in:

- [include/read_configs.h](/home/right/airslam_ws/src/AirSLAM_ROS2/include/read_configs.h)
- [demo/visual_odometry_online.cpp](/home/right/airslam_ws/src/AirSLAM_ROS2/demo/visual_odometry_online.cpp)
- [src/map_builder.cc](/home/right/airslam_ws/src/AirSLAM_ROS2/src/map_builder.cc)

### 8. Additional Dataset / Camera / Launch Assets

This fork adds supporting assets for deployment and testing:

- new camera configs:
  - `nus_thermal.yaml`
  - `nus_thermal_imu.yaml`
  - `profjosh.yaml`
  - `realsense_d435i.yaml`
- new map refinement launch files
- ROS2 visual odometry launch files
- updated RViz setup
- helper Python scripts for ONNX node adjustment

Related files:

- [configs/camera](/home/right/airslam_ws/src/AirSLAM_ROS2/configs/camera)
- [launch](/home/right/airslam_ws/src/AirSLAM_ROS2/launch)
- [python/onnx_check.py](/home/right/airslam_ws/src/AirSLAM_ROS2/python/onnx_check.py)
- [python/onnx_node_modify.py](/home/right/airslam_ws/src/AirSLAM_ROS2/python/onnx_node_modify.py)

## Practical One-Line Summary

Relative to upstream AirSLAM, this fork mainly adds:

1. ROS2 support
2. online stereo + IMU input
3. VIO integration for the online pipeline
4. Jetson AGX Orin / TensorRT deployment adaptation
5. RealSense online mode and supporting configs

## Notes

- This summary is based on local git history plus file-level comparison against upstream `sair-lab/AirSLAM`.
- It reflects what is verifiably present in this repository today, not only what appears in commit titles.

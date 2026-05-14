cmake_minimum_required(VERSION 3.16)
project(air_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "release")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
add_definitions(-w)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/tensorrtbuffer)
add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/DBoW2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)
find_package(G2O REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)
find_package(fmt REQUIRED)

include_directories(
  /usr/include/aarch64-linux-gnu/
  ${PROJECT_SOURCE_DIR} 
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${GFLAGS_INCLUDE_DIRS} 
  ${GLOG_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib SHARED
  src/g2o_optimization/vertex_line3d.cc
  src/g2o_optimization/vertex_vi_pose.cc
  src/g2o_optimization/vertex_imu.cc
  src/g2o_optimization/edge_project_point.cc
  src/g2o_optimization/edge_project_line.cc
  src/g2o_optimization/edge_imu.cc
  src/g2o_optimization/edge_relative_pose.cc
  src/g2o_optimization/g2o_optimization.cc
  src/bow/FSuperpoint.cc
  src/bow/database.cc
  src/super_point.cpp
  src/feature_detector.cc
  src/super_glue.cpp
  src/light_glue.cpp
  src/plnet.cpp
  src/utils.cc
  src/camera.cc
  src/imu.cc
  src/dataset.cc
  src/frame.cc
  src/point_matcher.cc
  src/mappoint.cc
  src/mapline.cc
  src/line_processor.cc
  src/ros_publisher.cc
  src/map.cc
  src/map_builder.cc
  src/map_refiner.cc
  src/map_user.cc
  src/timer.cc
  src/debug.cc
  src/ros_subscriber.cc
)

target_link_libraries(
  ${PROJECT_NAME}_lib
  nvinfer
  nvonnxparser
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${Boost_LIBRARIES}
  ${G2O_LIBRARIES}
  g2o_core
  ${GFLAGS_LIBRARIES} 
  ${GLOG_LIBRARIES}
  yaml-cpp
  tensorrtbuffer
  DBoW2
  fmt::fmt
  -lboost_serialization
)

ament_target_dependencies(
  ${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
)

add_executable(visual_odometry demo/visual_odometry.cpp)
target_link_libraries(visual_odometry ${PROJECT_NAME}_lib)
ament_target_dependencies(visual_odometry rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs cv_bridge image_transport tf2 tf2_ros)

add_executable(map_refinement demo/map_refinement.cpp)
target_link_libraries(map_refinement ${PROJECT_NAME}_lib)
ament_target_dependencies(map_refinement rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs cv_bridge image_transport tf2 tf2_ros)

add_executable(relocalization demo/relocalization.cpp)
target_link_libraries(relocalization ${PROJECT_NAME}_lib)
ament_target_dependencies(relocalization rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs cv_bridge image_transport tf2 tf2_ros)

add_executable(visual_odometry_online demo/visual_odometry_online.cpp)
target_link_libraries(visual_odometry_online ${PROJECT_NAME}_lib)
ament_target_dependencies(visual_odometry_online rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs cv_bridge image_transport tf2 tf2_ros)

# add_executable(test_feature demo/test_feature.cpp)
# target_link_libraries(test_feature ${PROJECT_NAME}_lib)
# ament_target_dependencies(test_feature rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs cv_bridge image_transport tf2 tf2_ros)

install(DIRECTORY
  launch
  configs
  output
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
visual_odometry
  visual_odometry_online
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_lib tensorrtbuffer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

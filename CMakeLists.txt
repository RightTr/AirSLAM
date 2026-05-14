cmake_minimum_required(VERSION 3.16)
project(air_slam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "release")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
add_definitions(-w)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROS_EDITION "${ROS_EDITION}" CACHE STRING "Target ROS edition: ROS1 or ROS2")
if(NOT ROS_EDITION)
  set(ROS_EDITION "ROS2")
endif()
string(TOUPPER "${ROS_EDITION}" ROS_EDITION)

if(ROS_EDITION STREQUAL "ROS1")
  add_compile_definitions(USE_ROS1)
  set(BUILD_WITH_ROS1 ON)
elseif(ROS_EDITION STREQUAL "ROS2")
  add_compile_definitions(USE_ROS2)
  set(BUILD_WITH_ROS2 ON)
else()
  message(FATAL_ERROR "Unsupported ROS_EDITION='${ROS_EDITION}'. Use ROS1 or ROS2.")
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/tensorrtbuffer)
add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/DBoW2)

set(ROS_DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  image_transport
)

if(BUILD_WITH_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    ${ROS_DEPENDENCIES}
  )
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_lib
    CATKIN_DEPENDS roscpp tf ${ROS_DEPENDENCIES}
  )
else()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  foreach(ros_dep ${ROS_DEPENDENCIES})
    find_package(${ros_dep} REQUIRED)
  endforeach()
endif()

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
  ${G2O_INCLUDE_DIR}
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

if(BUILD_WITH_ROS1)
  include_directories(${catkin_INCLUDE_DIRS})
endif()

set(AIR_SLAM_SOURCES
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

add_library(${PROJECT_NAME}_lib SHARED ${AIR_SLAM_SOURCES})

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

if(BUILD_WITH_ROS1)
  target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})
else()
  ament_target_dependencies(
    ${PROJECT_NAME}_lib
    rclcpp
    tf2
    tf2_ros
    ${ROS_DEPENDENCIES}
  )
endif()

add_executable(visual_odometry demo/visual_odometry.cpp)
target_link_libraries(visual_odometry ${PROJECT_NAME}_lib)

add_executable(map_refinement demo/map_refinement.cpp)
target_link_libraries(map_refinement ${PROJECT_NAME}_lib)

add_executable(relocalization demo/relocalization.cpp)
target_link_libraries(relocalization ${PROJECT_NAME}_lib)

add_executable(visual_odometry_online demo/visual_odometry_online.cpp)
target_link_libraries(visual_odometry_online ${PROJECT_NAME}_lib)

if(BUILD_WITH_ROS2)
  ament_target_dependencies(visual_odometry rclcpp tf2 tf2_ros ${ROS_DEPENDENCIES})
  ament_target_dependencies(map_refinement rclcpp tf2 tf2_ros ${ROS_DEPENDENCIES})
  ament_target_dependencies(relocalization rclcpp tf2 tf2_ros ${ROS_DEPENDENCIES})
  ament_target_dependencies(visual_odometry_online rclcpp tf2 tf2_ros ${ROS_DEPENDENCIES})
endif()

install(DIRECTORY
  launch
  configs
  output
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  visual_odometry
  map_refinement
  relocalization
  visual_odometry_online
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_lib tensorrtbuffer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_WITH_ROS2)
  ament_package()
endif()

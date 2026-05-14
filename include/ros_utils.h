#ifndef ROS_UTILS_H_
#define ROS_UTILS_H_

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <Eigen/Geometry>

#if !defined(USE_ROS1) && !defined(USE_ROS2)
#define USE_ROS2
#endif

#ifdef USE_ROS1
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

using RosNodePtr = std::shared_ptr<ros::NodeHandle>;
using TimeType = ros::Time;
using RateType = ros::Rate;
using HeaderMsg = std_msgs::Header;
using ImageMsg = sensor_msgs::Image;
using ImuMsg = sensor_msgs::Imu;
using PointMsg = geometry_msgs::Point;
using PoseMsg = geometry_msgs::Pose;
using PoseArrayMsg = geometry_msgs::PoseArray;
using PoseStampedMsg = geometry_msgs::PoseStamped;
using QuaternionMsg = geometry_msgs::Quaternion;
using TransformStampedMsg = geometry_msgs::TransformStamped;
using PathMsg = nav_msgs::Path;
using OdometryMsg = nav_msgs::Odometry;
using PointCloud2Msg = sensor_msgs::PointCloud2;
using MarkerMsg = visualization_msgs::Marker;
using ColorMsg = std_msgs::ColorRGBA;
using ImageMsgConstPtr = ImageMsg::ConstPtr;
using ImuMsgConstPtr = ImuMsg::ConstPtr;

namespace std_msgs { namespace msg {
using Header = ::std_msgs::Header;
using ColorRGBA = ::std_msgs::ColorRGBA;
} }

namespace sensor_msgs { namespace msg {
using Image = ::sensor_msgs::Image;
using Imu = ::sensor_msgs::Imu;
using PointCloud2 = ::sensor_msgs::PointCloud2;
} }

namespace geometry_msgs { namespace msg {
using Point = ::geometry_msgs::Point;
using Pose = ::geometry_msgs::Pose;
using PoseArray = ::geometry_msgs::PoseArray;
using PoseStamped = ::geometry_msgs::PoseStamped;
using Quaternion = ::geometry_msgs::Quaternion;
using TransformStamped = ::geometry_msgs::TransformStamped;
} }

namespace nav_msgs { namespace msg {
using Path = ::nav_msgs::Path;
using Odometry = ::nav_msgs::Odometry;
} }

namespace visualization_msgs { namespace msg {
using Marker = ::visualization_msgs::Marker;
} }

template<typename MsgT>
using RosPublisher = ros::Publisher;

template<typename MsgT>
using RosSubscription = ros::Subscriber;

class RosTransformBroadcaster {
public:
  explicit RosTransformBroadcaster(const RosNodePtr& node = nullptr) {
    (void)node;
  }

  void sendTransform(const TransformStampedMsg& msg) {
    tf::StampedTransform tf_msg;
    tf::transformStampedMsgToTF(msg, tf_msg);
    broadcaster_.sendTransform(tf_msg);
  }

private:
  tf::TransformBroadcaster broadcaster_;
};

inline void ros_init(int argc, char **argv, const std::string& node_name) {
  ros::init(argc, argv, node_name);
}

inline RosNodePtr make_ros_node(const std::string& node_name) {
  (void)node_name;
  return std::make_shared<ros::NodeHandle>();
}

inline bool ros_ok() {
  return ros::ok();
}

inline void ros_shutdown() {
  ros::shutdown();
}

inline void ros_spin_some(const RosNodePtr& node) {
  (void)node;
  ros::spinOnce();
}

template<typename T>
inline void ros_declare_parameter(const RosNodePtr& node, const std::string& name, const T& default_value) {
  (void)node;
  (void)name;
  (void)default_value;
}

template<typename T>
inline void ros_get_parameter(const RosNodePtr& node, const std::string& name, T& value, const T& default_value = T()) {
  node->param<T>(name, value, default_value);
}

inline TimeType to_ros_time(double seconds) {
  return ros::Time(seconds);
}

inline double from_ros_time(const ros::Time& stamp) {
  return stamp.toSec();
}

inline TimeType ros_now(const RosNodePtr& node) {
  (void)node;
  return ros::Time::now();
}

template<typename MsgT>
inline RosPublisher<MsgT> create_publisher(const RosNodePtr& node, const std::string& topic, std::size_t queue_size) {
  return node->advertise<MsgT>(topic, queue_size);
}

template<typename MsgT, typename CallbackT>
inline RosSubscription<MsgT> create_subscription(const RosNodePtr& node, const std::string& topic, std::size_t queue_size, CallbackT&& callback) {
  return node->subscribe<MsgT>(topic, queue_size, std::forward<CallbackT>(callback));
}

template<typename MsgT>
inline RosPublisher<MsgT> create_best_effort_publisher(const RosNodePtr& node, const std::string& topic, std::size_t queue_size) {
  return create_publisher<MsgT>(node, topic, queue_size);
}

template<typename MsgT>
inline void publish_message(const RosPublisher<MsgT>& publisher, const MsgT& msg) {
  publisher.publish(msg);
}

template<typename MsgT>
inline void shutdown_publisher(RosPublisher<MsgT>& publisher) {
  publisher.shutdown();
}

#elif defined(USE_ROS2)
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

using RosNodePtr = rclcpp::Node::SharedPtr;
using TimeType = rclcpp::Time;
using RateType = rclcpp::Rate;
using HeaderMsg = std_msgs::msg::Header;
using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;
using PointMsg = geometry_msgs::msg::Point;
using PoseMsg = geometry_msgs::msg::Pose;
using PoseArrayMsg = geometry_msgs::msg::PoseArray;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using QuaternionMsg = geometry_msgs::msg::Quaternion;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;
using PathMsg = nav_msgs::msg::Path;
using OdometryMsg = nav_msgs::msg::Odometry;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using MarkerMsg = visualization_msgs::msg::Marker;
using ColorMsg = std_msgs::msg::ColorRGBA;
using ImageMsgConstPtr = ImageMsg::ConstSharedPtr;
using ImuMsgConstPtr = ImuMsg::ConstSharedPtr;

template<typename MsgT>
using RosPublisher = typename rclcpp::Publisher<MsgT>::SharedPtr;

template<typename MsgT>
using RosSubscription = typename rclcpp::Subscription<MsgT>::SharedPtr;

class RosTransformBroadcaster {
public:
  explicit RosTransformBroadcaster(const RosNodePtr& node) : broadcaster_(node) {}

  void sendTransform(const TransformStampedMsg& msg) {
    broadcaster_.sendTransform(msg);
  }

private:
  tf2_ros::TransformBroadcaster broadcaster_;
};

inline void ros_init(int argc, char **argv, const std::string& node_name) {
  (void)node_name;
  rclcpp::init(argc, argv);
}

inline RosNodePtr make_ros_node(const std::string& node_name) {
  return rclcpp::Node::make_shared(node_name);
}

inline bool ros_ok() {
  return rclcpp::ok();
}

inline void ros_shutdown() {
  rclcpp::shutdown();
}

inline void ros_spin_some(const RosNodePtr& node) {
  rclcpp::spin_some(node);
}

template<typename T>
inline void ros_declare_parameter(const RosNodePtr& node, const std::string& name, const T& default_value) {
  node->declare_parameter<T>(name, default_value);
}

template<typename T>
inline void ros_get_parameter(const RosNodePtr& node, const std::string& name, T& value, const T& default_value = T()) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  value = node->get_parameter(name).get_value<T>();
}

template<>
inline void ros_get_parameter<float>(const RosNodePtr& node, const std::string& name, float& value, const float& default_value) {
  if (!node->has_parameter(name)) {
    node->declare_parameter<double>(name, static_cast<double>(default_value));
  }
  value = static_cast<float>(node->get_parameter(name).get_value<double>());
}

inline TimeType to_ros_time(double seconds) {
  const int32_t sec = static_cast<int32_t>(std::floor(seconds));
  const uint32_t nanosec = static_cast<uint32_t>((seconds - sec) * 1e9);
  return rclcpp::Time(sec, nanosec);
}

inline double from_ros_time(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

inline double from_ros_time(const rclcpp::Time& stamp) {
  return stamp.seconds();
}

inline TimeType ros_now(const RosNodePtr& node) {
  return node->get_clock()->now();
}

template<typename MsgT>
inline RosPublisher<MsgT> create_publisher(const RosNodePtr& node, const std::string& topic, std::size_t queue_size) {
  return node->create_publisher<MsgT>(topic, rclcpp::QoS(queue_size));
}

template<typename MsgT, typename CallbackT>
inline RosSubscription<MsgT> create_subscription(const RosNodePtr& node, const std::string& topic, std::size_t queue_size, CallbackT&& callback) {
  return node->create_subscription<MsgT>(topic, rclcpp::QoS(queue_size), std::forward<CallbackT>(callback));
}

template<typename MsgT>
inline RosPublisher<MsgT> create_best_effort_publisher(const RosNodePtr& node, const std::string& topic, std::size_t queue_size) {
  return node->create_publisher<MsgT>(topic, rclcpp::QoS(queue_size).best_effort());
}

template<typename MsgT>
inline void publish_message(const RosPublisher<MsgT>& publisher, const MsgT& msg) {
  publisher->publish(msg);
}

template<typename MsgT>
inline void shutdown_publisher(RosPublisher<MsgT>& publisher) {
  publisher.reset();
}

#endif

inline QuaternionMsg quaternion_from_rotation_matrix(const Eigen::Matrix3d& rotation) {
  Eigen::Quaterniond q(rotation);
  QuaternionMsg msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

#endif  // ROS_UTILS_H_

#ifndef ROS2_PUBLISHER_H_
#define ROS2_PUBLISHER_H_

#include <map>
#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "utils.h"
#include "read_configs.h"
#include "thread_publisher.h"

// #include "air_slam_interfaces/msg/feature_match.hpp"
// #include "air_slam_interfaces/msg/feature_message.hpp"
// #include "air_slam_interfaces/msg/frame_pose_message.hpp"
// #include "air_slam_interfaces/msg/keyframe_message.hpp"
// #include "air_slam_interfaces/msg/map_message.hpp"
// #include "air_slam_interfaces/msg/map_line_message.hpp"
// #include "air_slam_interfaces/msg/reloc_message.hpp"
// #include "air_slam_interfaces/msg/points_on_line.hpp"

// using FeatureMessage_ros = air_slam_interfaces::msg::FeatureMessage;
// using FeatureMessagePtr_ros = std::shared_ptr<FeatureMessage_ros>;
// using FeatureMessageConstPtr_ros = std::shared_ptr<const FeatureMessage_ros>;

// using FramePoseMessage_ros = air_slam_interfaces::msg::FramePoseMessage;
// using FramePoseMessagePtr_ros = std::shared_ptr<FramePoseMessage_ros>;
// using FramePoseMessageConstPtr_ros = std::shared_ptr<const FramePoseMessage_ros>;

// using KeyframeMessage_ros = air_slam_interfaces::msg::KeyframeMessage;
// using KeyframeMessagePtr_ros = std::shared_ptr<KeyframeMessage_ros>;
// using KeyframeMessageConstPtr_ros = std::shared_ptr<const KeyframeMessage_ros>;

// using MapMessage_ros = air_slam_interfaces::msg::MapMessage;
// using MapMessagePtr_ros = std::shared_ptr<MapMessage_ros>;
// using MapMessageConstPtr_ros = std::shared_ptr<const MapMessage_ros>;

// using MapLineMessage_ros = air_slam_interfaces::msg::MapLineMessage;
// using MapLineMessagePtr_ros = std::shared_ptr<MapLineMessage_ros>;
// using MapLineMessageConstPtr_ros = std::shared_ptr<const MapLineMessage_ros>;

// using RelocMessage_ros = air_slam_interfaces::msg::RelocMessage;
// using RelocMessagePtr_ros = std::shared_ptr<RelocMessage_ros>;
// using RelocMessageConstPtr_ros = std::shared_ptr<const RelocMessage_ros>;

enum FeatureMessageType {
  VOFeature = 0,
  RelocFeature = 1
};

struct FeatureMessage{
  double time;
  cv::Mat image;
  cv::Mat key_image;
  int frame_id;
  int keyframe_id;
  std::vector<bool> inliers;
  std::vector<cv::KeyPoint> keyframe_keypoints;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<Eigen::Vector4d> lines;
  std::vector<int> line_track_ids;
  std::vector<std::map<int, double>> points_on_lines;
  std::vector<cv::DMatch> matches;
  FeatureMessageType fm_type;
};
typedef std::shared_ptr<FeatureMessage> FeatureMessagePtr;
typedef std::shared_ptr<const FeatureMessage> FeatureMessageConstPtr;

struct FramePoseMessage{
  double time;
  Eigen::Matrix4d pose;
};
typedef std::shared_ptr<FramePoseMessage> FramePoseMessagePtr;
typedef std::shared_ptr<const FramePoseMessage> FramePoseMessageConstPtr;

struct KeyframeMessage{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double time;
  std::vector<double> times;
  std::vector<int> ids;
  std::vector<Eigen::Matrix4d> poses;
};
typedef std::shared_ptr<KeyframeMessage> KeyframeMessagePtr;
typedef std::shared_ptr<const KeyframeMessage> KeyframeMessageConstPtr;

struct MapMessage{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  double time;
  bool reset;
  std::vector<int> ids;
  std::vector<Eigen::Vector3d> points;
};
typedef std::shared_ptr<MapMessage> MapMessagePtr;
typedef std::shared_ptr<const MapMessage> MapMessageConstPtr;

struct MapLineMessage{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  double time;
  bool reset;
  std::vector<int> ids;
  std::vector<Vector6d> lines;
};
typedef std::shared_ptr<MapLineMessage> MapLineMessagePtr;
typedef std::shared_ptr<const MapLineMessage> MapLineMessageConstPtr;

struct RelocMessage{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double map_scale;
  std::vector<double> times;
  std::vector<Eigen::Matrix4d> poses;
  std::vector<Eigen::Vector3d> mappoints;
};
typedef std::shared_ptr<RelocMessage> RelocMessagePtr;
typedef std::shared_ptr<const RelocMessage> RelocMessageConstPtr;

// FeatureMessageConstPtr ConvertFromROS(FeatureMessageConstPtr_ros &msg_ros);
// FramePoseMessageConstPtr ConvertFromROS(FramePoseMessageConstPtr_ros &msg_ros);
// KeyframeMessageConstPtr ConvertFromROS(KeyframeMessageConstPtr_ros &msg_ros);
// MapMessageConstPtr ConvertFromROS(MapMessageConstPtr_ros &msg_ros);
// MapLineMessageConstPtr ConvertFromROS(MapLineMessageConstPtr_ros &msg_ros);
// RelocMessageConstPtr ConvertFromROS(RelocMessageConstPtr_ros &msg_ros);

class Ros2Publisher {
public:
  Ros2Publisher(const RosPublisherConfig& ros_publisher_config, rclcpp::Node::SharedPtr node);

  void PublishFeature(FeatureMessagePtr feature_message);
  void PublishFramePose(FramePoseMessagePtr frame_pose_message);
  void PublisheKeyframe(KeyframeMessagePtr keyframe_message);
  void PublishMap(MapMessagePtr map_message);
  void PublishMapLine(MapLineMessagePtr mapline_message);
  void PubRelocResults(RelocMessagePtr reloc_message);

  void Clear();
  void ShutDown();

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  
  RosPublisherConfig _config;

  // for publishing features
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _ros_feature_pub;
  ThreadPublisher<FeatureMessage> _feature_publisher;

  // for publishing frame
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _ros_frame_pose_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_latest_odometry;

  // for publishing keyframes
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _ros_keyframe_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _ros_path_pub;
  std::map<int, int> _keyframe_id_to_index;
  geometry_msgs::msg::PoseArray _ros_keyframe_array;
  nav_msgs::msg::Path _ros_path;

  // for publishing mappoints
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _ros_map_pub;
  std::unordered_map<int, int> _mappoint_id_to_index;
  sensor_msgs::msg::PointCloud2 _ros_mappoints;

  // for publishing maplines
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _ros_mapline_pub;
  std::unordered_map<int, int> _mapline_id_to_index;
  visualization_msgs::msg::Marker _ros_maplines;

  // for publishing relocalization results
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _ros_reloc_traj_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _ros_reloc_pose_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _ros_reloc_mpts_pub;
  visualization_msgs::msg::Marker _ros_reloc_traj;
};
typedef std::shared_ptr<Ros2Publisher> Ros2PublisherPtr;

#endif  // ROS2_PUBLISHER_H_
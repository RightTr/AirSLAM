#ifndef ROS2_PUBLISHER_H_
#define ROS2_PUBLISHER_H_

#include <map>
#include <vector>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "utils.h"
#include "read_configs.h"
#include "ros_utils.h"
#include "thread_publisher.h"

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

class RosPublisher {
public:
  RosPublisher(const RosPublisherConfig& ros_publisher_config, RosNodePtr node);

  void PublishFeature(FeatureMessagePtr feature_message);
  void PublishFramePose(FramePoseMessagePtr frame_pose_message);
  void PublisheKeyframe(KeyframeMessagePtr keyframe_message);
  void PublishMap(MapMessagePtr map_message);
  void PublishMapLine(MapLineMessagePtr mapline_message);
  void PubRelocResults(RelocMessagePtr reloc_message);

  void Clear();
  void ShutDown();

private:
  std::unique_ptr<RosTransformBroadcaster> _tf_broadcaster;
  RosNodePtr _node;
  
  RosPublisherConfig _config;

  // for publishing features
  RosPublisher<ImageMsg> _ros_feature_pub;
  ThreadPublisher<FeatureMessage> _feature_publisher;

  // for publishing frame
  RosPublisher<PoseStampedMsg> _ros_frame_pose_pub;
  RosPublisher<OdometryMsg> _pub_latest_odometry;

  // for publishing keyframes
  RosPublisher<PoseArrayMsg> _ros_keyframe_pub;
  RosPublisher<PathMsg> _ros_path_pub;
  std::map<int, int> _keyframe_id_to_index;
  PoseArrayMsg _ros_keyframe_array;
  PathMsg _ros_path;

  // for publishing mappoints
  RosPublisher<PointCloud2Msg> _ros_map_pub;
  std::unordered_map<int, int> _mappoint_id_to_index;
  PointCloud2Msg _ros_mappoints;

  // for publishing maplines
  RosPublisher<MarkerMsg> _ros_mapline_pub;
  std::unordered_map<int, int> _mapline_id_to_index;
  MarkerMsg _ros_maplines;

  // for publishing relocalization results
  RosPublisher<MarkerMsg> _ros_reloc_traj_pub;
  RosPublisher<PoseStampedMsg> _ros_reloc_pose_pub;
  RosPublisher<MarkerMsg> _ros_reloc_mpts_pub;
  MarkerMsg _ros_reloc_traj;
};
typedef std::shared_ptr<RosPublisher> RosPublisherPtr;

#endif  // ROS2_PUBLISHER_H_

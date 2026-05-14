#include "ros_publisher.h"

#include <Eigen/Geometry>

#include "utils.h"

RosPublisher::RosPublisher(const RosPublisherConfig& ros_publisher_config, RosNodePtr node): _config(ros_publisher_config), _node(node)
{

  if(_config.feature){
    _ros_feature_pub = create_publisher<ImageMsg>(node, _config.feature_topic, 10);
    std::function<void(const FeatureMessageConstPtr&)> publish_feature_function = 
        [&](const FeatureMessageConstPtr& feature_message){
      cv::Mat result;
      if(feature_message->fm_type == FeatureMessageType::RelocFeature){
        result = DrawFeatures(feature_message->image, feature_message->keypoints, feature_message->lines, true);
      }else{
        cv::Mat drawed_image = DrawFeatures(feature_message->image, feature_message->keypoints, 
            feature_message->lines, true);

        result = DrawMatches(feature_message->key_image, drawed_image, 
            feature_message->keyframe_keypoints, feature_message->keypoints, feature_message->matches);

        cv::copyMakeBorder(result, result, 50, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 255));
        cv::putText(result, "Feature Detection", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        std::string keyframe_info = "Keyframe Num : " + std::to_string(feature_message->keyframe_id);
        cv::putText(result, keyframe_info, cv::Point(10+drawed_image.cols, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        std::string frame_info = "Current Frame Id: " + std::to_string(feature_message->frame_id);
        cv::putText(result, frame_info, cv::Point(10+drawed_image.cols*2, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
      }

      auto ros_feature_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result).toImageMsg();
      ros_feature_msg->header.stamp = to_ros_time(feature_message->time);
      publish_message(_ros_feature_pub, *ros_feature_msg);
    };
  }

  if(_config.frame_pose){
    _ros_frame_pose_pub = create_publisher<PoseStampedMsg>(node, _config.frame_pose_topic, 10);
    _pub_latest_odometry = create_publisher<OdometryMsg>(node, _config.frame_odometry_topic, 1000);
    _tf_broadcaster = std::make_unique<RosTransformBroadcaster>(node);

    std::function<void(const FramePoseMessageConstPtr&)> publish_frame_pose_function = 
        [&](const FramePoseMessageConstPtr& frame_pose_message){
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = to_ros_time(frame_pose_message->time);
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose.position.x = frame_pose_message->pose(0, 3);
      pose_stamped.pose.position.y = frame_pose_message->pose(1, 3);
      pose_stamped.pose.position.z = frame_pose_message->pose(2, 3);
      Eigen::Quaterniond q(frame_pose_message->pose.block<3, 3>(0, 0));
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();
      publish_message(_ros_frame_pose_pub, pose_stamped);

      nav_msgs::msg::Odometry odometry;
      odometry.header.stamp = to_ros_time(frame_pose_message->time);
      odometry.header.frame_id = "map";
      odometry.pose.pose.position.x = frame_pose_message->pose(0, 3);
      odometry.pose.pose.position.y = frame_pose_message->pose(1, 3);
      odometry.pose.pose.position.z = frame_pose_message->pose(2, 3);
      odometry.pose.pose.orientation.x = q.x();
      odometry.pose.pose.orientation.y = q.y();
      odometry.pose.pose.orientation.z = q.z();
      odometry.pose.pose.orientation.w = q.w();
      publish_message(_pub_latest_odometry, odometry);
      
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = pose_stamped.header.stamp;
      transform.header.frame_id = "map";
      transform.child_frame_id = "camera";

      transform.transform.translation.x = frame_pose_message->pose(0,3);
      transform.transform.translation.y = frame_pose_message->pose(1,3);
      transform.transform.translation.z = frame_pose_message->pose(2,3);

      transform.transform.rotation.x = q.x();
      transform.transform.rotation.y = q.y();
      transform.transform.rotation.z = q.z();
      transform.transform.rotation.w = q.w();

      _tf_broadcaster->sendTransform(transform);
    };
  }

  if(_config.keyframe){
    _ros_keyframe_pub = create_publisher<PoseArrayMsg>(node, _config.keyframe_topic, 10);
    _ros_keyframe_array.header.frame_id = "map";

    _ros_path_pub = create_publisher<PathMsg>(node, _config.path_topic, 10);
    _ros_path.header.frame_id = "map";

    std::function<void(const KeyframeMessageConstPtr&)> publish_keyframe_function = 
        [&](const KeyframeMessageConstPtr& keyframe_message){
      _ros_keyframe_array.header.stamp = to_ros_time(keyframe_message->time);
      _ros_path.header.stamp = to_ros_time(keyframe_message->time);

      std::map<int, int>::iterator it;
      for(int i = 0; i < keyframe_message->ids.size(); i++){
        int keyframe_id = keyframe_message->ids[i];

        geometry_msgs::msg::Pose pose;
        pose.position.x = keyframe_message->poses[i](0, 3);
        pose.position.y = keyframe_message->poses[i](1, 3);
        pose.position.z = keyframe_message->poses[i](2, 3);
        Eigen::Quaterniond q(keyframe_message->poses[i].block<3, 3>(0, 0));
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = to_ros_time(keyframe_message->times[i]);
        pose_stamped.pose = pose;
        
        it = _keyframe_id_to_index.find(keyframe_id);
        if(it == _keyframe_id_to_index.end()){
          _ros_keyframe_array.poses.push_back(pose);
          _ros_path.poses.push_back(pose_stamped);
          _keyframe_id_to_index[keyframe_id] = _ros_keyframe_array.poses.size()-1;
        }else{
          int idx = it->second;
          _ros_keyframe_array.poses[idx] = pose;
          _ros_path.poses[idx] = pose_stamped;
        }
      }
      publish_message(_ros_keyframe_pub, _ros_keyframe_array);
      publish_message(_ros_path_pub, _ros_path);
    };
  }

  
  if (_config.map){
    _ros_map_pub = create_publisher<PointCloud2Msg>(node, _config.map_topic, 1);

    sensor_msgs::msg::PointCloud2 _ros_mappoints;
    _ros_mappoints.header.frame_id = "map";

    std::function<void(const MapMessageConstPtr &)> publish_map_function =
        [&](const MapMessageConstPtr &map_message)
        {
          _ros_mappoints.header.stamp = to_ros_time(map_message->time);

          _ros_mappoints.height = 1;
          _ros_mappoints.width = map_message->points.size();

          sensor_msgs::PointCloud2Modifier modifier(_ros_mappoints);
          modifier.setPointCloud2FieldsByString(1, "xyz");
          modifier.resize(map_message->points.size());

          sensor_msgs::PointCloud2Iterator<float> iter_x(_ros_mappoints, "x");
          sensor_msgs::PointCloud2Iterator<float> iter_y(_ros_mappoints, "y");
          sensor_msgs::PointCloud2Iterator<float> iter_z(_ros_mappoints, "z");

          for (size_t i = 0; i < map_message->points.size(); ++i, ++iter_x, ++iter_y, ++iter_z)
          {
              const auto &pt = map_message->points[i];
              *iter_x = pt(0);
              *iter_y = pt(1);
              *iter_z = pt(2);
          }

          publish_message(_ros_map_pub, _ros_mappoints);
        };

    // for maplines
    _ros_mapline_pub = create_publisher<MarkerMsg>(node, _config.mapline_topic, 1);
    _ros_maplines.header.stamp = ros_now(node);
    _ros_maplines.header.frame_id = "map"; 
    _ros_maplines.pose.position.x = 0;
    _ros_maplines.pose.position.y = 0;
    _ros_maplines.pose.position.z = 0;
    _ros_maplines.pose.orientation.x = 0;
    _ros_maplines.pose.orientation.y = 0;
    _ros_maplines.pose.orientation.z = 0;
    _ros_maplines.pose.orientation.w = 1.0;
    _ros_maplines.type = visualization_msgs::msg::Marker::LINE_LIST;
    _ros_maplines.scale.x = 0.05;  
    _ros_maplines.color.b = 1.0;
    _ros_maplines.color.a = 1.0;

    std::function<void(const MapLineMessageConstPtr&)> publish_mapline_function = 
        [&](const MapLineMessageConstPtr& mapline_message){
      _ros_maplines.header.stamp = to_ros_time(mapline_message->time);

      std::unordered_map<int, int>::iterator it;
      for(int i = 0; i < mapline_message->ids.size(); i++){
        int mapline_id = mapline_message->ids[i];
        it = _mapline_id_to_index.find(mapline_id);
        if(it == _mapline_id_to_index.end()){
          geometry_msgs::msg::Point point1, point2;
          point1.x = mapline_message->lines[i](0);
          point1.y = mapline_message->lines[i](1);
          point1.z = mapline_message->lines[i](2);
          point2.x = mapline_message->lines[i](3);
          point2.y = mapline_message->lines[i](4);
          point2.z = mapline_message->lines[i](5);
          _ros_maplines.points.push_back(point1);
          _ros_maplines.points.push_back(point2);

          std_msgs::msg::ColorRGBA color;
          Eigen::Vector3d color_vector;
          GenerateColor(mapline_id, color_vector);
          color.r = 0.0;
          color.g = 0.0;
          color.b = 1.0;   
          color.a = 1.0;   
          _ros_maplines.colors.push_back(color);
          _ros_maplines.colors.push_back(color);
          _mapline_id_to_index[mapline_id] = _ros_maplines.points.size()-2;
        }else{
          int idx = it->second;
          _ros_maplines.points[idx].x = mapline_message->lines[i](0);
          _ros_maplines.points[idx].y = mapline_message->lines[i](1);
          _ros_maplines.points[idx].z = mapline_message->lines[i](2);
          _ros_maplines.points[idx+1].x = mapline_message->lines[i](3);
          _ros_maplines.points[idx+1].y = mapline_message->lines[i](4);
          _ros_maplines.points[idx+1].z = mapline_message->lines[i](5);
        }
      }
      publish_message(_ros_mapline_pub, _ros_maplines);
    };
  }

  if(_config.reloc){
    // for relocalization trajectory
    _ros_reloc_traj_pub = create_publisher<MarkerMsg>(node, _config.reloc_topic+"/trajectory", 1);
    _ros_reloc_traj.header.stamp = ros_now(node);
    _ros_reloc_traj.header.frame_id = "map"; 
    _ros_reloc_traj.pose.position.x = 0;
    _ros_reloc_traj.pose.position.y = 0;
    _ros_reloc_traj.pose.position.z = 0;
    _ros_reloc_traj.pose.orientation.x = 0;
    _ros_reloc_traj.pose.orientation.y = 0;
    _ros_reloc_traj.pose.orientation.z = 0;
    _ros_reloc_traj.pose.orientation.w = 1.0;
    _ros_reloc_traj.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    // _ros_reloc_traj.scale.x = 0.8;  
    // _ros_reloc_traj.scale.y = 0.8;
    // _ros_reloc_traj.scale.z = 0.8;

    _ros_reloc_traj.color.a = 0.5; 
    _ros_reloc_traj.color.r = 0.0;
    _ros_reloc_traj.color.g = 1.0;
    _ros_reloc_traj.color.b = 0.0;  

    std::function<void(const RelocMessageConstPtr&)> publish_reloc_traj_function = 
        [&](const RelocMessageConstPtr& reloc_message){
      _ros_reloc_traj.scale.x = 0.8 * reloc_message->map_scale;  
      _ros_reloc_traj.scale.y = 0.8 * reloc_message->map_scale;
      _ros_reloc_traj.scale.z = 0.8 * reloc_message->map_scale;

      int idx = reloc_message->poses.size() - 1;
      if(idx < 0){
        return;
      }
      geometry_msgs::msg::Point p;
      p.x = reloc_message->poses[idx](0, 3);
      p.y = reloc_message->poses[idx](1, 3);
      p.z = reloc_message->poses[idx](2, 3);
      _ros_reloc_traj.points.push_back(p);

      publish_message(_ros_reloc_traj_pub, _ros_reloc_traj);
    };


    // for current pose
    _ros_reloc_pose_pub = create_publisher<PoseStampedMsg>(node, _config.reloc_topic+"/pose", 10);
    std::function<void(const RelocMessageConstPtr&)> publish_reloc_pose_function = 
        [&](const RelocMessageConstPtr& reloc_message){
      int idx = reloc_message->times.size() - 1;
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = to_ros_time(reloc_message->times[idx]);
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose.position.x = reloc_message->poses[idx](0, 3);
      pose_stamped.pose.position.y = reloc_message->poses[idx](1, 3);
      pose_stamped.pose.position.z = reloc_message->poses[idx](2, 3);
      Eigen::Quaterniond q(reloc_message->poses[idx].block<3, 3>(0, 0));
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();
      publish_message(_ros_reloc_pose_pub, pose_stamped);
    };

    // for matches
    _ros_reloc_mpts_pub = create_publisher<MarkerMsg>(node, _config.reloc_topic+"/matches", 1);
    std::function<void(const RelocMessageConstPtr&)> publish_reloc_mpts_function = 
        [&](const RelocMessageConstPtr& reloc_message){
      int idx = reloc_message->times.size() - 1;
      visualization_msgs::msg::Marker ros_reloc_mpts;
      ros_reloc_mpts.header.stamp = to_ros_time(reloc_message->times[idx]);
      ros_reloc_mpts.header.frame_id = "map"; 
      ros_reloc_mpts.pose.position.x = 0;
      ros_reloc_mpts.pose.position.y = 0;
      ros_reloc_mpts.pose.position.z = 0;
      ros_reloc_mpts.pose.orientation.x = 0;
      ros_reloc_mpts.pose.orientation.y = 0;
      ros_reloc_mpts.pose.orientation.z = 0;
      ros_reloc_mpts.pose.orientation.w = 1.0;
      ros_reloc_mpts.type = visualization_msgs::msg::Marker::LINE_LIST;
      ros_reloc_mpts.scale.x = 0.1 * reloc_message->map_scale;  
      ros_reloc_mpts.color.b = 1.0;
      ros_reloc_mpts.color.a = 0.5;

      geometry_msgs::msg::Point point1;
      point1.x = reloc_message->poses[idx](0, 3);
      point1.y = reloc_message->poses[idx](1, 3);
      point1.z = reloc_message->poses[idx](2, 3);

      for(int i = 0; i < reloc_message->mappoints.size(); i++){

        geometry_msgs::msg::Point point2;
        point2.x = reloc_message->mappoints[i](0);
        point2.y = reloc_message->mappoints[i](1);
        point2.z = reloc_message->mappoints[i](2);
        ros_reloc_mpts.points.push_back(point1);
        ros_reloc_mpts.points.push_back(point2);

        std_msgs::msg::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;   
        color.a = 1.0;   
        ros_reloc_mpts.colors.push_back(color);
        ros_reloc_mpts.colors.push_back(color);
      }
      publish_message(_ros_reloc_mpts_pub, ros_reloc_mpts);
    };
  }

}

void RosPublisher::PublishFeature(FeatureMessagePtr feature_message) {
    if (_ros_feature_pub) {
      cv::Mat result;
      if(feature_message->fm_type == FeatureMessageType::RelocFeature){
        result = DrawFeatures(feature_message->image, feature_message->keypoints, feature_message->lines, true);
      }else{
        cv::Mat drawed_image = DrawFeatures(feature_message->image, feature_message->keypoints, 
            feature_message->lines, true);

        result = DrawMatches(feature_message->key_image, drawed_image, 
            feature_message->keyframe_keypoints, feature_message->keypoints, feature_message->matches);

        cv::copyMakeBorder(result, result, 50, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 255));
        cv::putText(result, "Feature Detection", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        std::string keyframe_info = "Keyframe Num : " + std::to_string(feature_message->keyframe_id);
        cv::putText(result, keyframe_info, cv::Point(10+drawed_image.cols, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        std::string frame_info = "Current Frame Id: " + std::to_string(feature_message->frame_id);
        cv::putText(result, frame_info, cv::Point(10+drawed_image.cols*2, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
      }

      auto ros_feature_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result).toImageMsg();
      ros_feature_msg->header.stamp = to_ros_time(feature_message->time);
      publish_message(_ros_feature_pub, *ros_feature_msg);
    }
}

void RosPublisher::PublishFramePose(FramePoseMessagePtr frame_pose_message) {
    if (_ros_frame_pose_pub) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = to_ros_time(frame_pose_message->time);
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose.position.x = frame_pose_message->pose(0, 3);
      pose_stamped.pose.position.y = frame_pose_message->pose(1, 3);
      pose_stamped.pose.position.z = frame_pose_message->pose(2, 3);
      Eigen::Quaterniond q(frame_pose_message->pose.block<3, 3>(0, 0));
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();
      publish_message(_ros_frame_pose_pub, pose_stamped);

      nav_msgs::msg::Odometry odometry;
      odometry.header.stamp = to_ros_time(frame_pose_message->time);
      odometry.header.frame_id = "map";
      odometry.pose.pose.position.x = frame_pose_message->pose(0, 3);
      odometry.pose.pose.position.y = frame_pose_message->pose(1, 3);
      odometry.pose.pose.position.z = frame_pose_message->pose(2, 3);
      odometry.pose.pose.orientation.x = q.x();
      odometry.pose.pose.orientation.y = q.y();
      odometry.pose.pose.orientation.z = q.z();
      odometry.pose.pose.orientation.w = q.w();
      publish_message(_pub_latest_odometry, odometry);
      
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = pose_stamped.header.stamp;
      transform.header.frame_id = "map";
      transform.child_frame_id = "camera";

      transform.transform.translation.x = frame_pose_message->pose(0,3);
      transform.transform.translation.y = frame_pose_message->pose(1,3);
      transform.transform.translation.z = frame_pose_message->pose(2,3);

      transform.transform.rotation.x = q.x();
      transform.transform.rotation.y = q.y();
      transform.transform.rotation.z = q.z();
      transform.transform.rotation.w = q.w();

      _tf_broadcaster->sendTransform(transform);
    }
}

void RosPublisher::PublisheKeyframe(KeyframeMessagePtr keyframe_message) {
    if (_ros_keyframe_pub) {
      _ros_keyframe_array.header.stamp = to_ros_time(keyframe_message->time);
      _ros_path.header.stamp = to_ros_time(keyframe_message->time);

      std::map<int, int>::iterator it;
      for(int i = 0; i < keyframe_message->ids.size(); i++){
        int keyframe_id = keyframe_message->ids[i];

        geometry_msgs::msg::Pose pose;
        pose.position.x = keyframe_message->poses[i](0, 3);
        pose.position.y = keyframe_message->poses[i](1, 3);
        pose.position.z = keyframe_message->poses[i](2, 3);
        Eigen::Quaterniond q(keyframe_message->poses[i].block<3, 3>(0, 0));
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = to_ros_time(keyframe_message->times[i]);
        pose_stamped.pose = pose;
        
        it = _keyframe_id_to_index.find(keyframe_id);
        if(it == _keyframe_id_to_index.end()){
          _ros_keyframe_array.poses.push_back(pose);
          _ros_path.poses.push_back(pose_stamped);
          _keyframe_id_to_index[keyframe_id] = _ros_keyframe_array.poses.size()-1;
        }else{
          int idx = it->second;
          _ros_keyframe_array.poses[idx] = pose;
          _ros_path.poses[idx] = pose_stamped;
        }
        publish_message(_ros_keyframe_pub, _ros_keyframe_array);
        publish_message(_ros_path_pub, _ros_path);
    }
  }
}

void RosPublisher::PublishMap(MapMessagePtr map_message) {
    if (_ros_map_pub) {
      _ros_mappoints.header.stamp = to_ros_time(map_message->time);
      _ros_mappoints.header.frame_id = "map"; 

      _ros_mappoints.height = 1;
      _ros_mappoints.width = map_message->points.size();

      sensor_msgs::PointCloud2Modifier modifier(_ros_mappoints);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(map_message->points.size());

      sensor_msgs::PointCloud2Iterator<float> iter_x(_ros_mappoints, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(_ros_mappoints, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(_ros_mappoints, "z");

      for (size_t i = 0; i < map_message->points.size(); ++i, ++iter_x, ++iter_y, ++iter_z)
      {
          const auto &pt = map_message->points[i];
          *iter_x = pt(0);
          *iter_y = pt(1);
          *iter_z = pt(2);
      }

      publish_message(_ros_map_pub, _ros_mappoints);
    }
}

void RosPublisher::PublishMapLine(MapLineMessagePtr mapline_message) {
    if (_ros_mapline_pub) {
      _ros_maplines.header.stamp = to_ros_time(mapline_message->time);
      _ros_maplines.header.frame_id = "map"; 

      std::unordered_map<int, int>::iterator it;
      for(int i = 0; i < mapline_message->ids.size(); i++){
        int mapline_id = mapline_message->ids[i];
        it = _mapline_id_to_index.find(mapline_id);
        if(it == _mapline_id_to_index.end()){
          geometry_msgs::msg::Point point1, point2;
          point1.x = mapline_message->lines[i](0);
          point1.y = mapline_message->lines[i](1);
          point1.z = mapline_message->lines[i](2);
          point2.x = mapline_message->lines[i](3);
          point2.y = mapline_message->lines[i](4);
          point2.z = mapline_message->lines[i](5);
          _ros_maplines.points.push_back(point1);
          _ros_maplines.points.push_back(point2);

          std_msgs::msg::ColorRGBA color;
          Eigen::Vector3d color_vector;
          GenerateColor(mapline_id, color_vector);
          color.r = 0.0;
          color.g = 0.0;
          color.b = 1.0;   
          color.a = 1.0;   
          _ros_maplines.colors.push_back(color);
          _ros_maplines.colors.push_back(color);
          _mapline_id_to_index[mapline_id] = _ros_maplines.points.size()-2;
        }else{
          int idx = it->second;
          _ros_maplines.points[idx].x = mapline_message->lines[i](0);
          _ros_maplines.points[idx].y = mapline_message->lines[i](1);
          _ros_maplines.points[idx].z = mapline_message->lines[i](2);
          _ros_maplines.points[idx+1].x = mapline_message->lines[i](3);
          _ros_maplines.points[idx+1].y = mapline_message->lines[i](4);
          _ros_maplines.points[idx+1].z = mapline_message->lines[i](5);
        }
      }
      publish_message(_ros_mapline_pub, _ros_maplines);
    }
}

void RosPublisher::PubRelocResults(RelocMessagePtr reloc_message) {
    if (_ros_reloc_traj_pub) {
      _ros_reloc_traj.scale.x = 0.8 * reloc_message->map_scale;  
      _ros_reloc_traj.scale.y = 0.8 * reloc_message->map_scale;
      _ros_reloc_traj.scale.z = 0.8 * reloc_message->map_scale;

      int idx = reloc_message->poses.size() - 1;
      if(idx < 0){
        return;
      }
      geometry_msgs::msg::Point p;
      p.x = reloc_message->poses[idx](0, 3);
      p.y = reloc_message->poses[idx](1, 3);
      p.z = reloc_message->poses[idx](2, 3);
      _ros_reloc_traj.points.push_back(p);
      publish_message(_ros_reloc_traj_pub, _ros_reloc_traj);
    }
    if (_ros_reloc_pose_pub) {
      int idx = reloc_message->times.size() - 1;
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = to_ros_time(reloc_message->times[idx]);
      pose_stamped.header.frame_id = "map";
      pose_stamped.pose.position.x = reloc_message->poses[idx](0, 3);
      pose_stamped.pose.position.y = reloc_message->poses[idx](1, 3);
      pose_stamped.pose.position.z = reloc_message->poses[idx](2, 3);
      Eigen::Quaterniond q(reloc_message->poses[idx].block<3, 3>(0, 0));
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();
      publish_message(_ros_reloc_pose_pub, pose_stamped);
    }
    if (_ros_reloc_mpts_pub) {
      int idx = reloc_message->times.size() - 1;
      visualization_msgs::msg::Marker ros_reloc_mpts;
      ros_reloc_mpts.header.stamp = to_ros_time(reloc_message->times[idx]);
      ros_reloc_mpts.header.frame_id = "map"; 
      ros_reloc_mpts.pose.position.x = 0;
      ros_reloc_mpts.pose.position.y = 0;
      ros_reloc_mpts.pose.position.z = 0;
      ros_reloc_mpts.pose.orientation.x = 0;
      ros_reloc_mpts.pose.orientation.y = 0;
      ros_reloc_mpts.pose.orientation.z = 0;
      ros_reloc_mpts.pose.orientation.w = 1.0;
      ros_reloc_mpts.type = visualization_msgs::msg::Marker::LINE_LIST;
      ros_reloc_mpts.scale.x = 0.1 * reloc_message->map_scale;  
      ros_reloc_mpts.color.b = 1.0;
      ros_reloc_mpts.color.a = 0.5;

      geometry_msgs::msg::Point point1;
      point1.x = reloc_message->poses[idx](0, 3);
      point1.y = reloc_message->poses[idx](1, 3);
      point1.z = reloc_message->poses[idx](2, 3);

      for(int i = 0; i < reloc_message->mappoints.size(); i++){
        geometry_msgs::msg::Point point2;
        point2.x = reloc_message->mappoints[i](0);
        point2.y = reloc_message->mappoints[i](1);
        point2.z = reloc_message->mappoints[i](2);
        ros_reloc_mpts.points.push_back(point1);
        ros_reloc_mpts.points.push_back(point2);

        std_msgs::msg::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;   
        color.a = 1.0;   
        ros_reloc_mpts.colors.push_back(color);
        ros_reloc_mpts.colors.push_back(color);
      }
      publish_message(_ros_reloc_mpts_pub, ros_reloc_mpts);
    }
}

void RosPublisher::Clear() {
  _keyframe_id_to_index.clear();
  _ros_keyframe_array.poses.clear();
  _ros_path.poses.clear();

  _mappoint_id_to_index.clear();
  _ros_mappoints.data.clear();

  _mapline_id_to_index.clear();
  _ros_maplines.points.clear();
  _ros_maplines.colors.clear();
}

void RosPublisher::ShutDown() {
  if (_config.feature) {
      shutdown_publisher(_ros_feature_pub);
  }
  if (_config.frame_pose) {
      shutdown_publisher(_ros_frame_pose_pub);
      shutdown_publisher(_pub_latest_odometry);
  }
  if (_config.keyframe) {
      shutdown_publisher(_ros_keyframe_pub);
      shutdown_publisher(_ros_path_pub);
  }
  if (_config.map) {
      shutdown_publisher(_ros_map_pub);
      shutdown_publisher(_ros_mapline_pub);
  }
  if (_config.reloc) {
      shutdown_publisher(_ros_reloc_traj_pub);
      shutdown_publisher(_ros_reloc_pose_pub);
      shutdown_publisher(_ros_reloc_mpts_pub);
  }
}

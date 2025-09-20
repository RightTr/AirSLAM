#include "ros2_subscriber.h"

Ros2Subscriber::Ros2Subscriber(const RosSubscriberConfig& ros_subscriber_config, 
    rclcpp::Node::SharedPtr node) : buffer(10), _config(ros_subscriber_config)
{
    ros_imgl_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        _config.left_topic, 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx_);
            _ros_img_left = *msg;
            last_left_time_ = msg->header.stamp.sec +
                              msg->header.stamp.nanosec * 1e-9;
            TryPushStereo();
        });

    ros_imgr_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        _config.right_topic, 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx_);
            _ros_img_right = *msg;
            last_right_time_ = msg->header.stamp.sec +
                               msg->header.stamp.nanosec * 1e-9;
            TryPushStereo();
        });
    
}

bool Ros2Subscriber::PopStereoFrame(StereoFrame &frame)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (buffer.IsEmpty())
        return false;  
    frame = buffer.Pop();
    return true;
}

void Ros2Subscriber::TryPushStereo()
{
    if (_ros_img_left.data.empty() || _ros_img_right.data.empty()) return;

    double dt = std::abs(last_left_time_ - last_right_time_);
    if (dt < _config.time_thresh) {  
        cv::Mat left = cv_bridge::toCvCopy(
                           std::make_shared<sensor_msgs::msg::Image>(_ros_img_left), "bgr8")
                           ->image;
        cv::Mat right = cv_bridge::toCvCopy(
                            std::make_shared<sensor_msgs::msg::Image>(_ros_img_right), "bgr8")
                            ->image;

        StereoFrame frame(left, right, last_left_time_);
        buffer.Push(frame);

        RCLCPP_INFO(rclcpp::get_logger("Ros2Subscriber"),
                    "Pushed stereo frame (time %.6f)", frame._timestamp);

        _ros_img_left.data.clear();
        _ros_img_right.data.clear();
    }
}

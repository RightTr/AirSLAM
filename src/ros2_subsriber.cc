#include "ros2_subscriber.h"
#include "rclcpp/qos.hpp"

auto qos = rclcpp::SensorDataQoS().best_effort();

Ros2Subscriber::Ros2Subscriber(const RosSubscriberConfig& ros_subscriber_config, 
    rclcpp::Node::SharedPtr node) : buffer(10), _config(ros_subscriber_config)
{
    ros_imgl_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        _config.left_topic, qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx_);
            _ros_img_left = *msg;
            last_left_time_ = msg->header.stamp.sec +
                              msg->header.stamp.nanosec * 1e-9;
            TryPushStereo();
        });

    ros_imgr_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        _config.right_topic, qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx_);
            _ros_img_right = *msg;
            last_right_time_ = msg->header.stamp.sec +
                               msg->header.stamp.nanosec * 1e-9;
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
    std::cout << "tstamp: " << dt << std::endl; 
    if (dt < 0.05 && dt > 0.02) { 
        std::cout << "Associated tstamp: " << dt << std::endl; 

        auto cv_ptr_left = cv_bridge::toCvCopy(_ros_img_left, _ros_img_left.encoding);
        auto cv_ptr_right = cv_bridge::toCvCopy(_ros_img_right, _ros_img_right.encoding);

        cv::Mat left, right;

        if (cv_ptr_left->image.channels() == 3 || cv_ptr_left->image.type() != CV_8U)
            cv::cvtColor(cv_ptr_left->image, left, cv::COLOR_BGR2GRAY);
        else
            left = cv_ptr_left->image.clone();

        if (cv_ptr_right->image.channels() == 3 || cv_ptr_right->image.type() != CV_8U)
            cv::cvtColor(cv_ptr_right->image, right, cv::COLOR_BGR2GRAY);
        else
            right = cv_ptr_right->image.clone();

        StereoFrame frame(left, right, last_left_time_);
        buffer.Push(frame);

        RCLCPP_INFO(rclcpp::get_logger("Ros2Subscriber"),
                    "Pushed stereo frame (time %.6f)", frame._timestamp);

        _ros_img_left.data.clear();
        _ros_img_right.data.clear();
    }
}

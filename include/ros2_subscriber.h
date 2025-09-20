#ifndef ROS2_SUBSCRIBER_H_
#define ROS2_SUBSCRIBER_H_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "imgbuffer.h"

class Ros2Subscriber
{
    public:
        Ros2Subscriber(const RosSubscriberConfig& ros_subscriber_config, rclcpp::Node::SharedPtr node);

        bool PopStereoFrame(StereoFrame &frame);

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_imgl_sub_;
        sensor_msgs::msg::Image _ros_img_left;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_imgr_sub_;
        sensor_msgs::msg::Image _ros_img_right;

        Imgbuffer buffer;
        RosSubscriberConfig _config;

        double last_left_time_{0.0}, last_right_time_{0.0};
        std::mutex mtx_;

        void Ros2Subscriber::TryPushStereo();
};
typedef std::shared_ptr<Ros2Subscriber> Ros2SubscriberPtr;

#endif
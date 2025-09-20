#include "ros2_subscriber.h"

Ros2Subscriber::Ros2Subscriber(const RosSubscriberConfig& ros_subscriber_config, rclcpp::Node::SharedPtr node)
{

    _ros_imgl_sub = node->create_publisher<sensor_msgs::msg::Image>(ros_subscriber_config.left_topic, rclcpp::QoS(10));

    _ros_imgr_sub = node->create_publisher<sensor_msgs::msg::Image>(ros_subscriber_config.left_topic, rclcpp::QoS(10));
}
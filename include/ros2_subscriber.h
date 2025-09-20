#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class Ros2Subscriber
{
    public:
        Ros2Subscriber(rclcpp::Node::SharedPtr node);

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_imgl_sub_;
        sensor_msgs::msg::Image _ros_img_left;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_imgr_sub_;
        sensor_msgs::msg::Image _ros_img_right;

};


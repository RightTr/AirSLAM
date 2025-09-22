#ifndef ROS2_SUBSCRIBER_H_
#define ROS2_SUBSCRIBER_H_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "buffer.h"
#include "read_configs.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/imu.hpp>
#include "imu.h"

class Ros2Subscriber
{
    public:
        Ros2Subscriber(const RosSubscriberConfig& ros_subscriber_config, rclcpp::Node::SharedPtr node);

        bool PopStereoFrame(StereoFrame &frame);

        bool PopImuDataBetween(double t0, double t1, std::vector<sensor_msgs::msg::Imu> &imu_measurements)

        bool PopImuData(sensor_msgs::msg::Imu &imu)

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_imgl_sub_;
        sensor_msgs::msg::Image _ros_img_left;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_imgr_sub_;
        sensor_msgs::msg::Image _ros_img_right;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ros_imu_sub_;
        sensor_msgs::msg::Imu _ros_imu_data;

        ImuBuffer imu_buffer_;
        ImgBuffer img_buffer_;

        RosSubscriberConfig _config;

        double last_left_time_{-1.0}, last_right_time_{-1.0};
        std::mutex img_mtx_;
        std::mutex imu_mtx_;
        
        void TryPushStereo();
};
typedef std::shared_ptr<Ros2Subscriber> Ros2SubscriberPtr;

#endif
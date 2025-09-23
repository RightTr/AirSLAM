#include "ros2_subscriber.h"
#include "rclcpp/qos.hpp"

auto qos = rclcpp::SensorDataQoS().best_effort();

Ros2Subscriber::Ros2Subscriber(const RosSubscriberConfig& ros_subscriber_config, 
    rclcpp::Node::SharedPtr node) : img_buffer_(ros_subscriber_config.img_buffer_size),
    imu_buffer_(ros_subscriber_config.imu_buffer_size), _config(ros_subscriber_config)
{
    ros_imgl_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        _config.left_topic, qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(img_mtx_);
            _ros_img_left = *msg;
            last_left_time_ = msg->header.stamp.sec +
                              msg->header.stamp.nanosec * 1e-9;
            TryPushStereo();
        });

    ros_imgr_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        _config.right_topic, qos,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(img_mtx_);
            _ros_img_right = *msg;
            last_right_time_ = msg->header.stamp.sec +
                               msg->header.stamp.nanosec * 1e-9;
        });

    ros_imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    _config.imu_topic, qos,
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(imu_mtx_);
        ImuData data = RosImu2ImuData(*msg);
        imu_buffer_.Push(data);
    });
}

bool Ros2Subscriber::PopStereoFrame(StereoFrame &frame)
{
    std::lock_guard<std::mutex> lock(img_mtx_);
    if (img_buffer_.IsEmpty())
        return false;  
    frame = img_buffer_.Pop();
    return true;
}

bool Ros2Subscriber::PopImuDataBetween(double t0, double t1, std::vector<ImuData> &imu_measurements)
{
    std::lock_guard<std::mutex> lock(imu_mtx_);
    imu_measurements.clear();

    if (imu_buffer_.Size() < 2) return false;

    while (imu_buffer_.Size() >= 2 && imu_buffer_.Front().timestamp < t0) {
        ImuData imu0 = imu_buffer_.Pop();
        ImuData imu1 = imu_buffer_.Front();  

        if (imu1.timestamp >= t0) {
            double alpha = (t0 - imu0.timestamp) / (imu1.timestamp - imu0.timestamp);
            ImuData interp;
            interp.timestamp = t0;
            interp.acc = (1 - alpha) * imu0.acc + alpha * imu1.acc;
            interp.gyr = (1 - alpha) * imu0.gyr + alpha * imu1.gyr;
            imu_measurements.push_back(interp);
            break;
        }
    }

    while (!imu_buffer_.IsEmpty()) {
        ImuData imu = imu_buffer_.Front();
        if (imu.timestamp <= t1) {
            imu_measurements.push_back(imu_buffer_.Pop());
        } else {
            break;
        }
    }

    if (imu_measurements.empty()) return false;

    if (imu_measurements.back().timestamp < t1 && imu_buffer_.Size() > 0) {
        ImuData imu0 = imu_measurements.back();
        ImuData imu1 = imu_buffer_.Front(); 

        if (imu1.timestamp > imu0.timestamp) {
            double alpha = (t1 - imu0.timestamp) / (imu1.timestamp - imu0.timestamp);
            ImuData interp;
            interp.timestamp = t1;
            interp.acc = (1 - alpha) * imu0.acc + alpha * imu1.acc;
            interp.gyr = (1 - alpha) * imu0.gyr + alpha * imu1.gyr;
            imu_measurements.push_back(interp);
        }
    }

    return imu_measurements.size() >= 2;
}

bool Ros2Subscriber::PopImuData(ImuData &imu)
{
    std::lock_guard<std::mutex> lock(imu_mtx_);
    if (imu_buffer_.IsEmpty())
        return false;
    imu = imu_buffer_.Pop();
    return true;
}

void Ros2Subscriber::TryPushStereo()
{
    if (_ros_img_left.data.empty() || _ros_img_right.data.empty()) return;

    double dt = std::abs(last_left_time_ - last_right_time_);
    std::cout << "tstamp: " << dt << std::endl; 
    if (dt < _config.img_timediff_max && dt > _config.img_timediff_min) { 
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
        img_buffer_.Push(frame);

        RCLCPP_INFO(rclcpp::get_logger("Ros2Subscriber"),
                    "Pushed stereo frame (time %.6f)", frame._timestamp);

        _ros_img_left.data.clear();
        _ros_img_right.data.clear();
    }
}

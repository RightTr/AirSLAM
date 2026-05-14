#include "ros_subscriber.h"

#include <iomanip>
#include <iostream>

RosSubscriber::RosSubscriber(const RosSubscriberConfig& ros_subscriber_config, 
    RosNodePtr node) : img_buffer_(ros_subscriber_config.img_buffer_size),
    imu_buffer_(ros_subscriber_config.imu_buffer_size), _config(ros_subscriber_config)
{
    ros_imgl_sub_ = create_subscription<ImageMsg>(
        node, _config.left_topic, _config.img_buffer_size,
        [this](const ImageMsgConstPtr& msg) {
            std::lock_guard<std::mutex> lock(img_mtx_);
            _ros_img_left = *msg;
            last_left_time_ = from_ros_time(msg->header.stamp);
            TryPushStereo();
        });

    ros_imgr_sub_ = create_subscription<ImageMsg>(
        node, _config.right_topic, _config.img_buffer_size,
        [this](const ImageMsgConstPtr& msg) {
            std::lock_guard<std::mutex> lock(img_mtx_);
            _ros_img_right = *msg;
            last_right_time_ = from_ros_time(msg->header.stamp);
        });

    ros_imu_sub_ = create_subscription<ImuMsg>(
    node, _config.imu_topic, _config.imu_buffer_size,
    [this](const ImuMsgConstPtr& msg) {
        std::lock_guard<std::mutex> lock(imu_mtx_);
        ImuData data = RosImu2ImuData(*msg);
        imu_buffer_.Push(data);
    });
}

bool RosSubscriber::PopStereoFrame(StereoFrame &frame)
{
    std::lock_guard<std::mutex> lock(img_mtx_);
    if (img_buffer_.IsEmpty())
        return false;  
    frame = img_buffer_.Pop();
    return true;
}

bool RosSubscriber::PopImuDataBetween(double t0, double t1, std::vector<ImuData> &imu_measurements)
{
    std::lock_guard<std::mutex> lock(imu_mtx_);
    imu_measurements.clear();

    if (imu_buffer_.Size() < 2) return false;
    std::cout << "Start to Pop Imu Data" << std::endl;

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

bool RosSubscriber::PopImuData(ImuData &imu)
{
    std::lock_guard<std::mutex> lock(imu_mtx_);
    if (imu_buffer_.IsEmpty())
        return false;
    imu = imu_buffer_.Pop();
    return true;
}

void RosSubscriber::TryPushStereo()
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

        std::cout << "Pushed stereo frame (time " << std::fixed << std::setprecision(6)
                  << frame._timestamp << ")" << std::endl;

        _ros_img_left.data.clear();
        _ros_img_right.data.clear();
    }
}

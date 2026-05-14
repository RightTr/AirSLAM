#ifndef ROS2_SUBSCRIBER_H_
#define ROS2_SUBSCRIBER_H_

#include "imu.h"
#include "buffer.h"
#include "read_configs.h"
#include "ros_utils.h"

class RosSubscriber
{
    public:
        RosSubscriber(const RosSubscriberConfig& ros_subscriber_config, RosNodePtr node);

        bool PopStereoFrame(StereoFrame &frame);

        bool PopImuDataBetween(double t0, double t1, std::vector<ImuData> &imu_measurements);

        bool PopImuData(ImuData &imu);

    private:
        RosSubscription<ImageMsg> ros_imgl_sub_;
        ImageMsg _ros_img_left;

        RosSubscription<ImageMsg> ros_imgr_sub_;
        ImageMsg _ros_img_right;

        RosSubscription<ImuMsg> ros_imu_sub_;
        ImuMsg _ros_imu_data;

        ImuBuffer imu_buffer_;
        ImgBuffer img_buffer_;

        RosSubscriberConfig _config;

        double last_left_time_{-1.0}, last_right_time_{-1.0};
        std::mutex img_mtx_;
        std::mutex imu_mtx_;
        
        void TryPushStereo();
};
typedef std::shared_ptr<RosSubscriber> RosSubscriberPtr;

#endif

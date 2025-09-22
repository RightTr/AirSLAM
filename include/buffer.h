#ifndef BUFFER_H_
#define BUFFER_H_

#include "queue.h"
#include <opencv2/opencv.hpp>
#include <mutex>
#include <condition_variable>
#include <sensor_msgs/msg/imu.hpp>
#include "imu.h"

struct StereoFrame
{
    cv::Mat _left;
    cv::Mat _right;
    double _timestamp;

    StereoFrame(const cv::Mat &left, const cv::Mat &right, double timestamp)
        : _left(left.clone()), _right(right.clone()), _timestamp(timestamp) {}
    
    StereoFrame() : _timestamp(0.0) {}
};

class ImgBuffer
{
    public:
        ImgBuffer(size_t capacity = 100) : queue_(capacity) {}

        void Push(const StereoFrame &frame)
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cond_full_.wait(lock, [this]() { return !queue_.IsFull(); });

            queue_.Push(frame);

            cond_empty_.notify_one();
        }

        StereoFrame Pop()
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cond_empty_.wait(lock, [this]() { return !queue_.IsEmpty(); });

            StereoFrame frame = queue_.Pop();

            cond_full_.notify_one();
            return frame;
        }

        bool TryPop(StereoFrame &frame)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (queue_.IsEmpty()) return false;
            frame = queue_.Pop();
            cond_full_.notify_one();
            return true;
        }

        bool IsEmpty() const
        {
            std::lock_guard<std::mutex> lock(mtx_);
            return queue_.IsEmpty();
        }

        int Size() const
        {
            std::lock_guard<std::mutex> lock(mtx_);
            return queue_.Size();
        }

    private:
        Queue<StereoFrame> queue_;   
        mutable std::mutex mtx_;
        std::condition_variable cond_full_;
        std::condition_variable cond_empty_;

};

class ImuBuffer
{
    public:
        ImuBuffer(size_t capacity = 200) : queue_(capacity) {}

        void Push(const ImuData &imu)
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cond_full_.wait(lock, [this]() { return !queue_.IsFull(); });

            queue_.Push(imu);

            cond_empty_.notify_one();
        }

        ImuData Pop()
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cond_empty_.wait(lock, [this]() { return !queue_.IsEmpty(); });

            auto imu = queue_.Pop();

            cond_full_.notify_one();
            return imu;
        }

        bool TryPop(ImuData &imu)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (queue_.IsEmpty()) return false;
            imu = queue_.Pop();
            cond_full_.notify_one();
            return true;
        }

        bool IsEmpty() const
        {
            std::lock_guard<std::mutex> lock(mtx_);
            return queue_.IsEmpty();
        }

        int Size() const
        {
            std::lock_guard<std::mutex> lock(mtx_);
            return queue_.Size();
        }

    private:
        Queue<ImuData> queue_;   
        mutable std::mutex mtx_;
        std::condition_variable cond_full_;
        std::condition_variable cond_empty_;
};

#endif
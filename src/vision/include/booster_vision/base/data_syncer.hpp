#pragma once

#include <deque>
#include <mutex>

#include <opencv2/opencv.hpp>

#include <booster_vision/base/pose.h>

namespace booster_vision {
template <typename T>
struct DataBlock {
    T data;
    double timestamp;

    DataBlock() :
        timestamp(0) {
    }
    DataBlock(const T &data, double timestamp) :
        data(data), timestamp(timestamp) {
    }
};

using ColorDataBlock = DataBlock<cv::Mat>;
using DepthDataBlock = DataBlock<cv::Mat>;
using PoseDataBlock = DataBlock<Pose>;

struct SyncedDataBlock {
    ColorDataBlock color_data;
    DepthDataBlock depth_data;
    PoseDataBlock pose_data;
};

template <int MaxLength, typename T, typename Allocator = std::allocator<T>>
class DataBuffer : public std::deque<T, Allocator> {
public:
    DataBuffer() :
        std::deque<T, Allocator>(MaxLength) {
    }
    void push_back(const T &value) {
        if (this->size() == MaxLength) {
            this->pop_front();
        }
        std::deque<T, Allocator>::push_back(value);
    }
};

const int kDepthBufferLength = 30;
const int kPoseBufferLength = 500;

class DataSyncer {
public:
    DataSyncer(bool enable_depth) :
        enable_depth_(enable_depth) {
    }

    void AddDepth(const DepthDataBlock &depth_data);
    void AddPose(const PoseDataBlock &pose_data);

    SyncedDataBlock getSyncedDataBlock(const ColorDataBlock &color_data);

private:
    bool enable_depth_;
    std::mutex depth_buffer_mutex_;
    std::mutex pose_buffer_mutex_;

    using DepthBuffer = DataBuffer<kDepthBufferLength, DepthDataBlock>;
    using PoseBuffer = DataBuffer<kPoseBufferLength, PoseDataBlock>;

    DepthBuffer depth_buffer_;
    PoseBuffer pose_buffer_;
};

} // namespace booster_vision

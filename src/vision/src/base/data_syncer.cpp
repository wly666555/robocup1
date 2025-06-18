#include "booster_vision/base/data_syncer.hpp"

namespace booster_vision {

void DataSyncer::AddDepth(const DepthDataBlock &depth_data) {
    if (!enable_depth_) return;
    std::lock_guard<std::mutex> lock(depth_buffer_mutex_);
    depth_buffer_.push_back(depth_data);
}

void DataSyncer::AddPose(const PoseDataBlock &pose_data) {
    std::lock_guard<std::mutex> lock(pose_buffer_mutex_);
    pose_buffer_.push_back(pose_data);
}

SyncedDataBlock DataSyncer::getSyncedDataBlock(const ColorDataBlock &color_data) {
    SyncedDataBlock synced_data;
    synced_data.color_data = color_data;

    DepthBuffer depth_buffer_cp;
    {
        std::lock_guard<std::mutex> lock(depth_buffer_mutex_);
        depth_buffer_cp = depth_buffer_;
    }
    PoseBuffer pose_buffer_cp;
    {
        std::lock_guard<std::mutex> lock(pose_buffer_mutex_);
        pose_buffer_cp = pose_buffer_;
    }

    double color_timestamp = color_data.timestamp;
    std::cout << "color_timestamp: " << color_timestamp << std::endl;
    if (enable_depth_) {
        double smallest_depth_timestamp_diff = DBL_MAX;
        for (DepthBuffer::reverse_iterator it = depth_buffer_cp.rbegin(); it != depth_buffer_cp.rbegin() + kDepthBufferLength; ++it) {
            double diff = std::abs(it->timestamp - color_timestamp);
            if (diff < smallest_depth_timestamp_diff) {
                smallest_depth_timestamp_diff = diff;
                synced_data.depth_data = *it;
            } else {
                break;
            }
        }
        std::cout << "depth time diff: " << smallest_depth_timestamp_diff * 1000 << " ms" << std::endl;
    }

    double smallest_pose_timestamp_diff = DBL_MAX;
    for (PoseBuffer::reverse_iterator it = pose_buffer_cp.rbegin(); it != pose_buffer_cp.rbegin() + kPoseBufferLength; ++it) {
        double diff = std::abs(it->timestamp - color_timestamp);
        if (diff < smallest_pose_timestamp_diff) {
            smallest_pose_timestamp_diff = diff;
            synced_data.pose_data = *it;
        } else {
            break;
        }
    }
    std::cout << "pose time diff: " << smallest_pose_timestamp_diff * 1000 << " ms" << std::endl;
    return synced_data;
}

} // namespace booster_vision

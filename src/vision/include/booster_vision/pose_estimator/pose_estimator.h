#pragma once

#include <memory>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include "booster_vision/base/intrin.h"
#include "booster_vision/base/pose.h"
#include "booster_vision/model//detector.h"

namespace booster_vision {

class PoseEstimator {
public:
    using Ptr = std::shared_ptr<PoseEstimator>;
    PoseEstimator(const Intrinsics &intr) :
        intr_(intr) {
    }
    ~PoseEstimator() = default;

    virtual void Init(const YAML::Node &node){};

    virtual Pose EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb);
    virtual Pose EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &depth);

protected:
    Intrinsics intr_;
};

class BallPoseEstimator : public PoseEstimator {
public:
    BallPoseEstimator(const Intrinsics &intr) :
        PoseEstimator(intr) {
    }
    ~BallPoseEstimator() = default;

    void Init(const YAML::Node &node) override;
    Pose EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &depth) override;

private:
    bool use_depth_;
    float radius_;
    float downsample_leaf_size_;
    float cluster_distance_threshold_;
    float fitting_distance_threshold_;
};

class HumanLikePoseEstimator : public PoseEstimator {
public:
    HumanLikePoseEstimator(const Intrinsics &intr) :
        PoseEstimator(intr) {
    }
    ~HumanLikePoseEstimator() = default;

    void Init(const YAML::Node &node) override;
    Pose EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) override;
    Pose EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &depth) override;

private:
    bool use_depth_;
    float downsample_leaf_size_;
    float statistic_outlier_multiplier_;
    float fitting_distance_threshold_;
};

cv::Point3f CalculatePositionByIntersection(const Pose &p_eye2base, const cv::Point2f target_uv, const Intrinsics &intr);
} // namespace booster_vision

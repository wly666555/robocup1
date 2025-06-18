#pragma once
#include <string>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

namespace booster_vision {

struct Intrinsics {
    enum DistortionModel {
        kNone = 0,
        kBrownConrady = 1, // Opencv
        kInverseBrownConrady = 2
    };
    Intrinsics() = default;
    explicit Intrinsics(const YAML::Node &node);
    Intrinsics(const cv::Mat intr, const std::vector<float> &distortion_coeffs, const DistortionModel &model);
    Intrinsics(const cv::Mat intr) :
        Intrinsics(intr, std::vector<float>(), DistortionModel::kNone) {
    }
    Intrinsics(float fx, float fy, float cx, float cy, const std::vector<float> &distortion_coeffs, DistortionModel model);
    Intrinsics(float fx, float fy, float cx, float cy) :
        Intrinsics(fx, fy, cx, cy, std::vector<float>(), DistortionModel::kNone) {
    }

    cv::Point2f Project(const cv::Point3f &point) const;
    cv::Point3f BackProject(const cv::Point2f &point, float depth = 1.0) const;
    cv::Point2f UnDistort(const cv::Point2f &point) const;

    cv::Mat get_intrinsics_matrix() const {
        return (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }

    float fx = 0;
    float fy = 0;
    float cx = 0;
    float cy = 0;
    std::vector<float> distortion_coeffs = {};
    DistortionModel model = DistortionModel::kNone;
};

} // namespace booster_vision
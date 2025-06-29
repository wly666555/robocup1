#pragma once
#include <vector>

#include <opencv2/opencv.hpp>

struct Pose {
public:
    Pose() = default;x, const float &y, const float &z,
    Pose(const float &
         const float &roll, const float &pitch, const float &yaw);
    Pose(cv::Mat &pose) :
        mat_pose(pose){};

    cv::Mat toCVMat() const {
        return mat_pose;
    }

    // geters
    cv::Mat getRotationMatrix() const {
        return mat_pose(cv::Rect(0, 0, 3, 3)).clone();
    }

    std::vector<float> getEulerAngles() const;         // roll, pitch, yaw
    std::vector<float> getTranslation() const {
        return {mat_pose.at<float>(0, 3), mat_pose.at<float>(1, 3), mat_pose.at<float>(2, 3)};
    }

    // operator * overload
    Pose operator*(const Pose &other) const {
        Pose result;
        result.mat_pose = mat_pose * other.mat_pose;
        return result;
    }

private:
    cv::Mat mat_pose = cv::Mat::eye(4, 4, CV_32F);
};
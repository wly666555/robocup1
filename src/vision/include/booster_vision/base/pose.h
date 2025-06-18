#pragma once
#include <vector>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

// #include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace booster_vision {

struct Pose {
public:
    Pose() = default;
    Pose(const float &x, const float &y, const float &z,
         const float &roll, const float &pitch, const float &yaw);
    Pose(cv::Mat &pose) :
        mat_pose(pose){};
    Pose(float x, float y, float z,
         float qx, float qy, float qz, float qw);
    Pose(const geometry_msgs::msg::TransformStamped &msg);

    cv::Mat toCVMat() const {
        return mat_pose;
    }
    geometry_msgs::msg::TransformStamped toRosTFMsg();

    // geters
    cv::Mat getRotationMatrix() const {
        return mat_pose(cv::Rect(0, 0, 3, 3)).clone();
    }

    std::vector<double> getRotationQuaternion() const; // x,y,z,w
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

} // namespace booster_vision

namespace YAML {
// Specialize the convert template for cv::Mat
template <>
struct convert<booster_vision::Pose> {
    static bool decode(const Node &node, booster_vision::Pose &pose) {
        if (!node.IsSequence() || node.size() != 4) {
            return false; // Or throw an exception
        }
        cv::Mat mat = cv::Mat::zeros(4, 4, CV_32F);
        for (size_t i = 0; i < 4; ++i) {
            const auto &row = node[i];
            if (!row.IsSequence() || row.size() != 4) {
                return false; // Or throw an exception
            }
            for (size_t j = 0; j < 4; ++j) {
                mat.at<float>(i, j) = row[j].as<float>();
            }
        }
        pose = booster_vision::Pose(mat);
        return true;
    }
};
} // namespace YAML
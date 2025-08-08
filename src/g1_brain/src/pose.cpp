#include "pose.h"

Pose::Pose(const float &x, const float &y, const float &z,
           const float &roll, const float &pitch, const float &yaw) {
    cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                  0, cos(roll), -sin(roll),
                  0, sin(roll), cos(roll));

    cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch),
                  0, 1, 0,
                  -sin(pitch), 0, cos(pitch));

    cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0,
                  sin(yaw), cos(yaw), 0,
                  0, 0, 1);
    cv::Mat R = Rz * Ry * Rx;

    R.copyTo(mat_pose(cv::Rect(0, 0, 3, 3)));

    mat_pose.at<float>(0, 3) = x;
    mat_pose.at<float>(1, 3) = y;
    mat_pose.at<float>(2, 3) = z;
}


std::vector<float> Pose::getEulerAngles() const {
    cv::Mat R = getRotationMatrix();

    float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0));
    bool singular = sy < 1e-6; // If true, we're at a singularity

    float x, y, z; // Roll, Pitch, Yaw
    if (!singular) {
        x = atan2(R.at<float>(2, 1), R.at<float>(2, 2)); // Yaw
        y = atan2(-R.at<float>(2, 0), sy);               // Pitch
        z = atan2(R.at<float>(1, 0), R.at<float>(0, 0)); // Roll
    } else {
        x = atan2(-R.at<float>(1, 2), R.at<float>(1, 1)); // Yaw
        y = atan2(-R.at<float>(2, 0), sy);                // Pitch
        z = 0;                                            // Roll is set to 0 in singularity case
    }

    return {x, y, z};
}

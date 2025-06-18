#include "booster_vision/pose_estimator/pose_estimator.h"

#include "booster_vision/base/misc_utils.hpp"
#include "booster_vision/base/pointcloud_process.h"

namespace booster_vision {

cv::Point3f CalculatePositionByIntersection(const Pose &p_eye2base, const cv::Point2f target_uv, const Intrinsics &intr) {
    cv::Point3f normalized_point3d = intr.BackProject(target_uv);

    cv::Mat mat_obj_ray = (cv::Mat_<float>(3, 1) << normalized_point3d.x, normalized_point3d.y, normalized_point3d.z);
    cv::Mat mat_rot = p_eye2base.getRotationMatrix();
    cv::Mat mat_trans = p_eye2base.toCVMat().col(3).rowRange(0, 3);

    cv::Mat mat_rot_obj_ray = mat_rot * mat_obj_ray;

    float scale = -mat_trans.at<float>(2, 0) / mat_rot_obj_ray.at<float>(2, 0);

    cv::Mat mat_position = mat_trans + scale * mat_rot_obj_ray;
    return cv::Point3f(mat_position.at<float>(0, 0), mat_position.at<float>(1, 0), mat_position.at<float>(2, 0));
}

Pose PoseEstimator::EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) {
    // TODO(GW): add modification for cross class
    auto bbox = detection.bbox;
    cv::Point2f target_uv = cv::Point2f(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    cv::Point3f target_xyz = CalculatePositionByIntersection(p_eye2base, target_uv, intr_);
    return Pose(target_xyz.x, target_xyz.y, target_xyz.z, 0, 0, 0);
}

Pose PoseEstimator::EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &depth) {
    return Pose();
}

void BallPoseEstimator::Init(const YAML::Node &node) {
    use_depth_ = as_or<bool>(node["use_depth"], false);
    radius_ = as_or<float>(node["radius"], 0.109);
    downsample_leaf_size_ = as_or<float>(node["downsample_leaf_size"], 0.01);
    cluster_distance_threshold_ = as_or<float>(node["cluster_distance_threshold"], 0.01);
    fitting_distance_threshold_ = as_or<float>(node["fitting_distance_threshold"], 0.01);
}

Pose BallPoseEstimator::EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &depth) {
    if (!use_depth_ || depth.empty()) return Pose();

    auto pose = PoseEstimator::EstimateByColor(p_eye2base, detection, cv::Mat());
    if (pose.getTranslation()[0] > 3) return pose;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    CreatePointCloud(cloud, depth, cv::Mat(), detection.bbox, intr_);
    if (cloud->points.size() < 100) return pose;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    DownSamplePointCloud(downsampled_cloud, downsample_leaf_size_, cloud);
    if (downsampled_cloud->points.size() < 100) return pose;

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clustered_clouds;
    ClusterPointCloud(clustered_clouds, downsampled_cloud, cluster_distance_threshold_);

    if (clustered_clouds.empty()) return pose;
    for (auto cluster : clustered_clouds) {
        std::vector<float> sphere;
        float confidence;
        SphereFitting(sphere, confidence, cluster, fitting_distance_threshold_, radius_);
        if (confidence > 0.5) {
            pose = Pose(sphere[0], sphere[1], sphere[2], 0, 0, 0);
            break;
        }
    }
    return p_eye2base * pose;
}

void HumanLikePoseEstimator::Init(const YAML::Node &node) {
    use_depth_ = as_or<bool>(node["use_depth"], false);
    downsample_leaf_size_ = as_or<float>(node["downsample_leaf_size"], 0.01);
    statistic_outlier_multiplier_ = as_or<float>(node["statistic_outlier_multiplier"], 0.01);
    fitting_distance_threshold_ = as_or<float>(node["fitting_distance_threshold"], 0.01);
}

Pose HumanLikePoseEstimator::EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) {
    auto bbox = detection.bbox;
    cv::Point2f target_uv = cv::Point2f(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    cv::Point3f target_xyz = CalculatePositionByIntersection(p_eye2base, target_uv, intr_);
    return Pose(target_xyz.x, target_xyz.y, target_xyz.z, 0, 0, 0);
}

Pose HumanLikePoseEstimator::EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &depth) {
    if (!use_depth_ || depth.empty()) return Pose();

    auto pose = PoseEstimator::EstimateByColor(p_eye2base, detection, cv::Mat());
    if (pose.getTranslation()[0] > 3) return pose;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    CreatePointCloud(cloud, depth, cv::Mat(), detection.bbox, intr_);
    if (cloud->points.size() < 100) return pose;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    DownSamplePointCloud(downsampled_cloud, downsample_leaf_size_, cloud);
    if (downsampled_cloud->points.size() < 100) return pose;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloudNoiseRemoval(processed_cloud, 50, statistic_outlier_multiplier_, downsampled_cloud);
    if (processed_cloud->points.size() < 100) return pose;

    std::vector<float> plane_coeffs;
    float confidence;
    PlaneFitting(plane_coeffs, confidence, processed_cloud, fitting_distance_threshold_);

    // compute plane ray intersection
    auto bbox = detection.bbox;
    cv::Point2f target_uv = cv::Point2f(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    cv::Point3f normalized_point3d = intr_.BackProject(target_uv);

    float a = plane_coeffs[0];
    float b = plane_coeffs[1];
    float c = plane_coeffs[2];
    float d = plane_coeffs[3];
    float scale = -d / (normalized_point3d.x * a + normalized_point3d.y * b + c);

    pose = p_eye2base * Pose(normalized_point3d.x * scale, normalized_point3d.y * scale, normalized_point3d.z * scale, 0, 0, 0);
    return pose;
}

} // namespace booster_vision

#include "booster_vision/base/pointcloud_process.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace booster_vision {

void CreatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const cv::Mat &depth_image, const cv::Mat &rgb_image, const cv::Rect &bbox,
                      const booster_vision::Intrinsics &intrinsics) {
    for (int v = bbox.y; v < bbox.y + bbox.height; ++v) {
        for (int u = bbox.x; u < bbox.x + bbox.width; ++u) {
            float depth = depth_image.at<float>(v, u);
            if (!std::isnan(depth) && depth > 0) {
                auto cv_point = intrinsics.BackProject(cv::Point2f(u, v), depth);

                pcl::PointXYZRGB point;
                point.x = cv_point.x;
                point.y = cv_point.y;
                point.z = cv_point.z;
                point.b = rgb_image.at<cv::Vec3b>(v, u)[0];
                point.g = rgb_image.at<cv::Vec3b>(v, u)[1];
                point.r = rgb_image.at<cv::Vec3b>(v, u)[2];
                cloud->points.push_back(point);
            }
        }
    }
}

void CreatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const cv::Mat &depth_image, const cv::Mat &rgb_image,
                      const booster_vision::Intrinsics &intrinsics) {
    cv::Rect bbox(0, 0, depth_image.cols, depth_image.rows);
    CreatePointCloud(cloud, depth_image, rgb_image, bbox, intrinsics);
}

void DownSamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &processed_cloud, const float leaf_size,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size); // Adjust leaf size as needed
    voxel_filter.filter(*processed_cloud);
}

void PointCloudNoiseRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &processed_cloud, const int neighbour_count,
                            const float multiplier,
                            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(neighbour_count);      // Number of nearest neighbors to use
    sor.setStddevMulThresh(multiplier); // Standard deviation multiplier
    sor.filter(*processed_cloud);
}

void ClusterPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clustered_clouds, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                       const float cluster_distance_threshold) {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setInputCloud(cloud);
    ec.setClusterTolerance(cluster_distance_threshold); // Adjust tolerance as needed
    ec.setMinClusterSize(100);                          // Minimum number of points per cluster
    ec.setMaxClusterSize(125000);                       // Maximum number of points per cluster
    ec.setSearchMethod(tree);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    std::cout << "number of cluster: " << cluster_indices.size() << std::endl;
    // If no clusters found, return false
    if (cluster_indices.empty()) {
        clustered_clouds = {};
        return;
    }

    for (const auto &indices : cluster_indices) {
        std::cout << "cluster size: " << indices.indices.size() << std::endl;
        // if (indices.indices.size() < cloud->points.size() * 0.20) {
        //   std::cout << "cluster size: " << indices.indices.size() << " smaller than threshold, skip!" << std::endl;
        //   continue; // skip cluster with size smaller than fifth of the input cloud
        // }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int index : indices.indices) {
            cluster->points.push_back(cloud->points[index]);
        }

        clustered_clouds.push_back(cluster);
    }
}

void SphereFitting(std::vector<float> &sphere, float &confidence, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                   const float &dist_threshold, const float &radius_threshold) {
    sphere = {0, 0, 0, 0};
    confidence = 0;

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;

    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist_threshold); // Adjust distance threshold as needed

    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    if (inliers.indices.empty()) {
        return;
    }

    sphere = coefficients.values;
    if (std::abs(coefficients.values[3] - radius_threshold) > 0.02) {
        std::cout << "raidus " << coefficients.values[3] << " higher than expected " << radius_threshold << std::endl;
        std::cout << "center: " << sphere[0] << ", " << sphere[1] << ", " << sphere[2] << std::endl;
        return;
    }

    confidence = static_cast<float>(inliers.indices.size()) / cloud->points.size();
}

void PlaneFitting(std::vector<float> &plane, float &confidence,
                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const float &dist_threshold) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist_threshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    plane = {0, 0, 0, 0};
    confidence = 0;
    if (inliers->indices.size() < 100) {
        return;
    }

    std::cout << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;
    plane = coefficients->values;
    confidence = static_cast<float>(inliers->indices.size()) / cloud->size();
    std::cout << "inlier percentage: " << confidence << std::endl;

    auto point = cloud->points[inliers->indices[0]];
    // VisualizePointCloudandPlane(cloud, plane, point.x, point.y, point.z);
}

} // namespace booster_vision
#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/angles.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/ModelCoefficients.h>
#include <queue>
#include <pcl/segmentation/sac_segmentation.h>
#include <functional>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <chrono>
#include <GoSdk/GoSdk.h>
#include <kApi/kApi.h>

using namespace std;

class utils
{
public:
    utils() = default;
    ~utils() = default;
    static void visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    static void visualize(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
    static void visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

    static bool loadPCDFile(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>, const std::string& savepath);
    static Eigen::VectorXf fit_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int>& inliers, double th);
    static void fit_plane_perpendicular(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
    static void fit_line(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int>& inliers, Eigen::VectorXf &model_coefficients, double th);
    static void passthrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::string axis, float min, float max, bool negative);
    static void passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& axis, float min, float max, bool negative);
    static void radius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, float radius, int min_neighbors);
    static void EuclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointIndices> &cluster_indices, bool is_visualize = false);
    static void remove_3zero(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
    static void cal_dis(Eigen::VectorXf coefficients, pcl::PointXYZ point);
};

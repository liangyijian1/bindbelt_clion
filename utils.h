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
    utils() {};
    ~utils() {};
    static void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    static void visualize(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    static void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

    static bool loadPCDFile(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>, const std::string& savepath);
    static Eigen::VectorXf fit_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& inliers, double th);
    static void fit_plane_perpendicular(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    static void passthrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::string axis, float min, float max, bool negative);
    static void passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string axis, float min, float max, bool negative);
    static void radiustfilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, float radius, int min_neighbors);
    static void EuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointIndices> &cluster_indices);

};

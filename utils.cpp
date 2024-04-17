#include "utils.h"

void utils::visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer.addCoordinateSystem(200.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(chrono::microseconds(100000));
    }
}

void utils::visualize(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
{
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.setBackgroundColor(255.0, 255.0, 255.0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer.addCoordinateSystem(200.0);
    viewer.initCameraParameters();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(chrono::microseconds(100000));
    }
}

void utils::visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud show"));
    int v1 = 0;
    int v2 = 1;

    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1, 1, v2);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->setBackgroundColor(0, 0, 0, v2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud1, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> after_sac(cloud2, 0, 0, 255);

    viewer->addPointCloud(cloud1, color, "cloud", v1);
    viewer->addPointCloud(cloud2, after_sac, "plane cloud", v2);


    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(chrono::microseconds(10000));
    }
}

bool utils::loadPCDFile(const std::string& filename, pcl::PointCloud<pcl::PointXYZ> cloud,
                        const std::string& savepath = "")
{
    /*
    From .txt to .pcd
    */
    std::ifstream fs;
    fs.open(filename.c_str(), std::ios::binary);
    if (!fs.is_open() || fs.fail())
    {
        PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(), strerror(errno));
        fs.close();
        return (false);
    }
    std::string line;
    std::vector<std::string> st;

    while (!fs.eof())
    {
        std::getline(fs, line);
        if (line.empty())
            continue;
        boost::trim(line);
        boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

        if (st.size() != 3)
            continue;

        cloud.push_back(pcl::PointXYZ(float(atof(st[0].c_str())), float(atof(st[1].c_str())),
                                      float(atof(st[2].c_str()))));
    }
    fs.close();
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = true;
    if (!savepath.empty())
        pcl::io::savePCDFileASCII(savepath, cloud);
    return (true);
}

Eigen::VectorXf utils::fit_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int>& inliers, double th)
{
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
    ransac.setDistanceThreshold(th);
    ransac.setMaxIterations(200);
    ransac.setProbability(0.99);
    ransac.computeModel();
    ransac.getInliers(inliers);
    Eigen::VectorXf coef;
    ransac.getModelCoefficients(coef);
    return coef;
}

void utils::fit_plane_perpendicular(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
{
    double degree = 90.0;
    pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGBA>::Ptr model(
            new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGBA>(cloud));
    model->setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
    model->setEpsAngle(pcl::deg2rad(degree));
    pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac(model);
    ransac.setDistanceThreshold(10);
    ransac.computeModel();
    std::vector<int> plane_inliers;
    plane_inliers.clear();
    ransac.getInliers(plane_inliers);

    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud(cloud);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    inliers->indices = plane_inliers;
    extract.setIndices(inliers);
    extract.setNegative(false);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr show1(new pcl::PointCloud<pcl::PointXYZRGBA>);
    extract.filter(*show1);
    pcl::io::savePCDFileBinary("./point_soft_16_13_17_855_bindbelt_SAC.pcd", *show1);
}

void utils::passthrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::string axis, float min, float max,
                        bool negative = true)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(min, max);
    pass.setNegative(negative);
    pass.filter(*cloud_filtered);
}

void utils::passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& axis, float min, float max,
                        bool negative)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(min, max);
    pass.setNegative(negative);
    pass.filter(*cloud);
}

void utils::radius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, float radius, int min_neighbors)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius(min_neighbors);
    outrem.filter(*cloud_filtered);
}

void utils::EuclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointIndices> &cluster_indices, bool is_visualize)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(3);
    ec.setMinClusterSize(2);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);


    if (!is_visualize){
        //将cloud转换为PointXYZRGB
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud, *cloud_rgb);
        pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

        for (const auto& single_clusters : cluster_indices){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

            for (auto point : single_clusters.indices){

                cloud_cluster->push_back((*cloud_rgb)[point]);
            }
            auto R = rand() % 255;
            auto G = rand() % 255;
            auto B = rand() % 255;

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_cluster, R, G, B);
            viewer.addPointCloud(cloud_cluster, single_color, "cloud_cluster" + std::to_string(R) + std::to_string(G) + std::to_string(B));
        }

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(chrono::microseconds(100000));
        }
    }
}

void utils::fit_line(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, vector<int> &inliers, Eigen::VectorXf &model_coefficients, double th) {
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);
    ransac.setDistanceThreshold(th);
    ransac.setMaxIterations(200);
    ransac.setProbability(0.99);
    ransac.computeModel();
    ransac.getInliers(inliers);
    ransac.getModelCoefficients(model_coefficients);
}

void utils::remove_3zero(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (cloud->points[i].x == 0 && cloud->points[i].y == 0 && cloud->points[i].z == 0)
        {
            indices->indices.push_back(static_cast<int>(i));
        }
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
}

double utils::cal_dis(Eigen::VectorXf coefficients, pcl::PointXYZ point) {
    //计算点云中的点到直线的距离
    double a = coefficients[0];
    double b = coefficients[1];
    double c = coefficients[2];
    double d = coefficients[3];
    double e = coefficients[4];
    double f = coefficients[5];
    double x0 = point.x;
    double y0 = point.y;
    double z0 = point.z;
    return abs(a * x0 + b * y0 + c * z0 + d) / sqrt(a * a + b * b + c * c);

}

//
// Created by leihao on 2024/4/16.
//

#include "LMI.h"

void LMI::process_line() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr line1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr line2(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(R"(C:\Users\22692\Downloads\profile_2d_1712815245.pcd)", *cloud) == -1) {
        cout << "load failed" << '\n';
    }

    utils::remove_3zero(cloud, cloud_filter);
    utils::radius_filter(cloud_filter, 15, 8);
    //utils::visualize(cloud, cloud_filter);
    // 一次拟合
    Eigen::VectorXf line_coefficients;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    vector<int> line;
    indices->indices.clear();
    utils::fit_line(cloud_filter, line, line_coefficients, 4);
    // 去除拟合直线上的点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    indices->indices = line;
    extract.setInputCloud(cloud_filter);
    extract.setNegative(true);
    extract.setIndices(indices);
    extract.filter(*line1);
    // 二次拟合
    Eigen::VectorXf line_coefficients1;
    line.clear();
    indices->indices.clear();
    utils::fit_line(line1, line, line_coefficients1, 4);
    indices->indices = line;
    extract.setInputCloud(line1);
    extract.setNegative(true);
    extract.setIndices(indices);
    extract.filter(*line2);
    cout << "直线方程为：\n"
         << "   (x - " << line_coefficients1[0] << ") / " << line_coefficients1[3]
         << " = (y - " << line_coefficients1[1] << ") / " << line_coefficients1[4]
         << " = (z - " << line_coefficients1[2] << ") / " << line_coefficients1[5] << endl;
    // 可视化
    utils::visualize(cloud, line2);
}

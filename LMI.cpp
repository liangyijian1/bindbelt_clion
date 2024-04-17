//
// Created by leihao on 2024/4/16.
//

#include "LMI.h"

void LMI::process_line() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr line1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr line2(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(R"(C:\Users\22692\Downloads\profile_2d_1712815172.pcd)", *cloud) == -1) {
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
    utils::fit_line(cloud_filter, line, line_coefficients, 5);
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
    utils::fit_line(line1, line, line_coefficients1, 5);
    indices->indices = line;
    extract.setInputCloud(line1);
    extract.setNegative(true);
    extract.setIndices(indices);
    extract.filter(*line2);
    vector<double> dis;
    double max_value = 0;
    for (auto p : line2->points){
        double tmp_dis = utils::cal_dis(line_coefficients1, p);
        if (tmp_dis > max_value)
            max_value = tmp_dis;
        dis.push_back(tmp_dis);
    }
    cout << max_value << '\n';
    // 可视化
    utils::visualize(cloud, line2);
}

LMI::LMI() {
    cout << "initializing..." << '\n';
    api = kNULL;
    system = kNULL;
    sensor = kNULL;
    setup = kNULL;
    dataset = kNULL;
    data_obj = kNULL;

    // construct Gocator API Library
    if ((status = GoSdk_Construct(&api)) != kOK)
    {
        printf("Error: GoSdk_Construct:%d\n", status);
        return;
    }
    // construct GoSystem object
    if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
    {
        printf("Error: GoSystem_Construct:%d\n", status);
        return;
    }
    kIpAddress_Parse(&ip_address, "192.168.1.10");
    if ((status = GoSystem_FindSensorByIpAddress(system, &ip_address, &sensor)) != kOK)
    {
        printf("Error: GoSystem_FindSensor:%d\n", status);
        return;
    }
    if ((status = GoSensor_Connect(sensor)) != kOK)
    {
        printf("Error: GoSensor_Connect:%d\n", status);
        return;
    }
    if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
    {
        printf("Error: GoSensor_EnableData:%d\n", status);
        return;
    }
    setup = GoSensor_Setup(sensor);
    if ((status = GoSystem_Start(system)) != kOK)
    {
        printf("Error: GoSystem_Start:%d\n", status);
        return;
    }
}

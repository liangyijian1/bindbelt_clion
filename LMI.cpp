//
// Created by leihao on 2024/4/16.
//

#include "LMI.h"

void LMI::process_line() {
    if (GoSystem_ReceiveData(system, &dataset, 20000000) != kOK){
        GoDestroy(dataset);
        cout << "receive failed" << '\n';
    }
    else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        kSize go_data_set_count = GoDataSet_Count(dataset);
        for (int i = 0; i < go_data_set_count; ++i)
        {
            data_obj = GoDataSet_At(dataset, i);
            switch (GoDataMsg_Type(data_obj))
            {
                case GO_DATA_MESSAGE_TYPE_STAMP:
                    break;
                case GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE:
                {
                    GoResampledProfileMsg profileMsg = data_obj;

                    auto row_count = GoResampledProfileMsg_Count(profileMsg);
                    auto width = GoResampledProfileMsg_Width(profileMsg);
                    cloud->height = row_count;
                    cloud->width = width;
                    cloud->is_dense = false;
                    cloud->resize(width * row_count);

                    double x_resolution = static_cast<double>(GoResampledProfileMsg_XResolution(profileMsg)) / 1000000;
                    double z_resolution = static_cast<double>(GoResampledProfileMsg_ZResolution(profileMsg)) / 1000000; //0.0177
                    double x_offset = static_cast<double>(GoResampledProfileMsg_XOffset(profileMsg)) / 1000;
                    double z_offset = static_cast<double>(GoResampledProfileMsg_ZOffset(profileMsg)) / 1000; //80.435

                    for (unsigned int k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k)
                    {
                        short* data = GoResampledProfileMsg_At(profileMsg, 0);
                        for (unsigned int Index = 0; Index < GoResampledProfileMsg_Width(profileMsg); ++Index)
                        {
                            double x = x_offset + x_resolution * Index;
                            double z = z_offset + z_resolution * data[Index];
                            double y = 0;
                            //输入到cloud中
                            cloud->points[k * width + Index].x = static_cast<float>(x);
                            cloud->points[k * width + Index].y = static_cast<float>(y);
                            cloud->points[k * width + Index].z = static_cast<float>(z);
                        }
                    } // for (unsigned int k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k)

                    //utils::visualize(cloud);
                } // case GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE
            } // switch (GoDataMsg_Type(data_obj))
        } // for (int i = 0; i < go_data_set_count; ++i)

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr line1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr line2(new pcl::PointCloud<pcl::PointXYZ>);
        utils::passthrough(cloud, cloud_filter1, "z", 100, 1000, false);

        utils::remove_3zero(cloud_filter1, cloud_filter);
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
        for (auto p: line2->points) {
            double tmp_dis = utils::cal_dis(line_coefficients1, p);
            if (tmp_dis > max_value)
                max_value = tmp_dis;
            dis.push_back(tmp_dis);
        }
        cout << "Number of outer pixels: " << dis.size() << "\n" << "Max distance: " << max_value << '\n' <<
        "Box Status: " << (max_value > 150 && dis.size() > 10 ? "NG" : "OK") << '\n';
        // 可视化
        utils::visualize(cloud, line2);

    }

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

LMI::~LMI() {
    cout << "destroying..." << '\n';
    if ((status = GoSystem_Stop(system)) != kOK)
    {
        printf("Error: GoSystem_Stop:%d\n", status);
        return;
    }
    GoDestroy(dataset);
    GoDestroy(sensor);
    //GoDestroy(system);
    GoDestroy(api);
}

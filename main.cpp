#include "utils.h"
#include <cmath>
#include <filesystem>
#include <iostream>

namespace _pcl
{
    namespace fs = std::filesystem;

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointXYZRGBA PointTRGBA;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef pcl::PointCloud<PointTRGBA> PointCloudRGBA;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef pcl::PointCloud<PointTRGBA>::Ptr PointCloudRGBAPtr;

    bool LMI_is_OK(const PointCloudPtr& cloud)
    {
        PointCloudPtr cloud_plane1(new PointCloud);
        PointCloudPtr cloud_plane2(new PointCloud);
        PointCloudPtr tmp_cloud(new PointCloud);
        //utils::passthrough(cloud, tmp_cloud, "y", -5, 5, true); //去除平滑连接区域
        utils::passthrough(cloud, tmp_cloud, "x", -80, 50, false); //边缘裁剪
        //utils::passthrough(tmp_cloud, "y", -150, 150, false);
        //utils::passthrough(tmp_cloud, "z", 11, 270, false);
        pcl::io::savePCDFile(R"(C:\Users\10458\Downloads\surface1.pcd)", *tmp_cloud);

        //return true;

        pcl::PointIndices::Ptr nan_indices(new pcl::PointIndices);
        for (size_t i = 0; i < tmp_cloud->points.size(); ++i)
        {
            if (tmp_cloud->points[i].x == 0 && tmp_cloud->points[i].y == 0 && tmp_cloud->points[i].z == 0)
            {
                nan_indices->indices.push_back(i);
            }
        }

        //utils::visualize(cloud, tmp_cloud);
        //utils::radiustfilter(cloud, 5, 10);

        int th = 10;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(tmp_cloud);
        extract.setIndices(nan_indices);
        extract.setNegative(true);
        extract.filter(*tmp_cloud);

        std::vector<int> tmp;
        utils::fit_plane(tmp_cloud, tmp, th);
        pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
        plane_indices->indices = tmp;
        extract.setInputCloud(tmp_cloud);
        extract.setIndices(plane_indices);
        extract.setNegative(true);
        extract.filter(*cloud_plane1);

        std::vector<int> tmp1;
        utils::fit_plane(cloud_plane1, tmp1, th);
        pcl::PointIndices::Ptr plane_indices1(new pcl::PointIndices);
        plane_indices1->indices = tmp1;
        extract.setInputCloud(cloud_plane1);
        extract.setIndices(plane_indices1);
        extract.setNegative(true);
        extract.filter(*cloud_plane2);


        std::cout << cloud->points.size() << '\n';
        std::cout << cloud_plane2->points.size() << '\n';
        pcl::io::savePCDFile(R"(C:\Users\10458\Downloads\surface.pcd)", *cloud_plane2);
        utils::visualize(tmp_cloud, cloud_plane2);

        int res = cloud_plane2->points.size();
        bool tag = res < 200;

        cloud_plane2->clear();
        cloud_plane1->clear();

        return tag;
    }

    void LMI_receive_sync(int validPointCountTh)
    {
        kAssembly api = kNULL;
        kStatus status;
        GoSystem system = kNULL;
        GoSensor sensor = kNULL;
        GoSetup setup = kNULL;
        GoDataSet dataset = kNULL;
        GoDataMsg data_obj = kNULL;
        kIpAddress ip_address;
        char tmp = '\n';

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

        bool real_data_arrived = false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        while (!real_data_arrived)
        {
            if (GoSystem_ReceiveData(system, &dataset, 20000000) != kOK)
            {
                GoDestroy(dataset);
                cout << "receive failed" << '\n';
                break;
            }
            else
            {
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
                            int validPointCount = 0;
                            for (unsigned int k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k)
                            {
                                short* data = GoResampledProfileMsg_At(profileMsg, 0);
                                for (unsigned int Index = 0; Index < GoResampledProfileMsg_Width(profileMsg); ++Index)
                                {
                                    if (data[Index] != -32768) validPointCount++;
                                }
                            }

                            if (validPointCount > validPointCountTh)
                            {
                                GoSystem_Stop(system);
                                //std::this_thread::sleep_for(chrono::microseconds(1000));
                                //重启采样，设置为点云采集模式
                                if ((status = GoSetup_SetScanMode(setup, GO_MODE_SURFACE)) != kOK)
                                    printf("Error: GoSetup_SetScanMode:%d\n", status);
                                GoSystem_Start(system);
                                //std::this_thread::sleep_for(chrono::microseconds(1000));
                            }
                            cout << validPointCount << '\n';
                            break;
                        }

                        case GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE:
                        {
                            GoSurfaceMsg surface_msg = data_obj;
                            pcl::ExtractIndices<pcl::PointXYZ>::Ptr extract;
                            unsigned int row_count = GoSurfaceMsg_Length(surface_msg);
                            unsigned int width = GoSurfaceMsg_Width(surface_msg);
                            cloud->height = row_count;
                            cloud->width = width;
                            cloud->is_dense = false;
                            cloud->resize(width * row_count);

                            auto now = chrono::system_clock::now();
                            time_t time = chrono::system_clock::to_time_t(now);
                            auto filename = string(R"(C:\Users\10458\Downloads\)") + "surface_" + to_string(time) + ".pcd";
                            double x_resolution = static_cast<double>(GoSurfaceMsg_XResolution(surface_msg)) / 1000000;
                            double y_resolution = static_cast<double>(GoSurfaceMsg_YResolution(surface_msg)) / 1000000; //0.5
                            double z_resolution = static_cast<double>(GoSurfaceMsg_ZResolution(surface_msg)) / 1000000; //0.0177
                            double x_offset = static_cast<double>(GoSurfaceMsg_XOffset(surface_msg)) / 1000;
                            double y_offset = static_cast<double>(GoSurfaceMsg_YOffset(surface_msg)) / 1000;
                            double z_offset = static_cast<double>(GoSurfaceMsg_ZOffset(surface_msg)) / 1000; //80.435

                            for (unsigned int row_idx = 0; row_idx < row_count; row_idx++)
                            {
                                k16s* data = GoSurfaceMsg_RowAt(surface_msg, row_idx);
                                for (unsigned int colIdx = 0; colIdx < GoSurfaceMsg_Width(surface_msg); colIdx++)
                                {
                                    double x = x_offset + x_resolution * colIdx;
                                    double y = y_offset + y_resolution * row_idx;

                                    if (data[colIdx] != -32768)
                                    {
                                        double z = z_offset + z_resolution * data[colIdx];
                                        cloud->points[row_idx * width + colIdx].x = x;
                                        cloud->points[row_idx * width + colIdx].y = y;
                                        cloud->points[row_idx * width + colIdx].z = z;

                                    }
                                    else double z = -32768;
                                }
                            }

                            if (LMI_is_OK(cloud))
                                cout << "OK" << '\n';
                            else
                                cout << "NG" << '\n';

                            if (pcl::io::savePCDFileASCII(filename, *cloud) == -1) {
                                cout << "扫描数据部分缺失，请重新扫描" << '\n';
                            }

                            cloud->clear();
                            cloud_out->clear();

                            GoSystem_Stop(system);
                            //重启采样，设置为轮廓采集模式
                            if ((status = GoSetup_SetScanMode(setup, GO_MODE_PROFILE)) != kOK)
                                printf("Error: GoSetup_SetScanMode:%d\n", status);
                            GoSystem_Start(system);
                            break;
                        } // case GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE
                    } // switch (GoDataMsg_Type(data_obj))
                }
            } // else
            dataset = kNULL;
            data_obj = kNULL;
        } // while (!real_data_arrived)

        // 停止传感器
        if ((status = GoSystem_Stop(system)) != kOK)
        {
            printf("Error: GoSystem_Stop:%d\n", status);
            return;
        }
        // 注销系统
        GoDestroy(system);
        GoDestroy(api);
    }

    void LMI_receive_line() {
        kAssembly api = kNULL;
        kStatus status;
        GoSystem system = kNULL;
        GoSensor sensor = kNULL;
        GoSetup setup = kNULL;
        GoDataSet dataset = kNULL;
        GoDataMsg data_obj = kNULL;
        kIpAddress ip_address;
        char tmp = '\n';
        fstream f1, f2;

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

        bool real_data_arrived = false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

        if (GoSystem_ReceiveData(system, &dataset, 20000000) != kOK)
        {
            GoDestroy(dataset);
            cout << "receive failed" << '\n';
        }
        else
        {
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
                        int validPointCount = 0;

                        auto filename1 = string(R"(C:\Users\10458\Downloads\)") + "profile_3d_" + to_string(chrono::system_clock::to_time_t(chrono::system_clock::now())) + ".txt";
                        auto filename2 = string(R"(C:\Users\10458\Downloads\)") + "profile_2d_" + to_string(chrono::system_clock::to_time_t(chrono::system_clock::now())) + ".txt";
                        f1.open(filename1, ios::out | ios::app);
                        f2.open(filename2, ios::out | ios::app);
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
                                if (data[Index] != -32768)
                                {
                                    f1 << x << ' ' << 0 << ' ' << z << '\n';
                                    f2 << x << ' ' << z << '\n';
                                    validPointCount++;
                                }
                            }
                        }

                        cout << validPointCount << '\n';
                        f1.close();
                        f2.close();
                        break;
                    } //  GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE
                } // switch (GoDataMsg_Type(data_obj))
            }
        } // else
        dataset = kNULL;
        data_obj = kNULL;

        // 停止传感器
        if ((status = GoSystem_Stop(system)) != kOK)
        {
            printf("Error: GoSystem_Stop:%d\n", status);
            return;
        }
        // 注销系统
        GoDestroy(system);
        GoDestroy(api);

    }

    void LMI_Process_line(){
        //本地读取PCD
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile(R"(C:\Users\22692\Downloads\profile_2d_1712815172.pcd)", *cloud) == -1) {
            cout << "load failed" << '\n';
        }
        //欧式聚类
        std::vector<pcl::PointIndices> cluster_indices;
        utils::EuclideanCluster(cloud, cluster_indices);
        //可视化聚类结果


    }
}


int main(int argc, char* argv[])
{
    _pcl::LMI_Process_line();

    return 0;
}

//
// Created by leihao on 2024/4/16.
//

#ifndef BINDBELT_CLION_LMI_H
#define BINDBELT_CLION_LMI_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


class LMI {

public:
    static void process_line(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

};


#endif //BINDBELT_CLION_LMI_H

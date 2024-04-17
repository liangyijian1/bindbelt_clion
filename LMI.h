//
// Created by leihao on 2024/4/16.
//

#ifndef BINDBELT_CLION_LMI_H
#define BINDBELT_CLION_LMI_H

#include "utils.h"
#include <GoSdk/GoSdk.h>
#include <kApi/kApi.h>

class LMI {

public:
    LMI();
    ~LMI();
    void process_line();

private:
    kAssembly api;
    kStatus status;
    GoSystem system;
    GoSensor sensor;
    GoSetup setup;
    GoDataSet dataset;
    GoDataMsg data_obj;
    kIpAddress ip_address{};

};


#endif //BINDBELT_CLION_LMI_H

cmake_minimum_required(VERSION 3.28)
cmake_policy(SET CMP0144 NEW)
project(bindbelt_clion)


set(CMAKE_CXX_STANDARD 17)

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# LMI
file(GLOB LMI_LIB_FILES  "C:/Users/22692/Downloads/14400-6.1.32.12_SOFTWARE_GO_SDK/GO_SDK/lib/win64d/*.lib")
include_directories("C:/Users/22692/Downloads/14400-6.1.32.12_SOFTWARE_GO_SDK/GO_SDK/Platform/kApi")
include_directories("C:/Users/22692/Downloads/14400-6.1.32.12_SOFTWARE_GO_SDK/GO_SDK/Gocator/GoSdk")
link_directories("C:/Users/22692/Downloads/14400-6.1.32.12_SOFTWARE_GO_SDK/GO_SDK/lib/win64d")


add_executable(bindbelt_clion main.cpp
        utils.cpp
        utils.h
        LMI.cpp
        LMI.h
)
target_link_libraries (bindbelt_clion ${PCL_LIBRARIES})
target_link_libraries(bindbelt_clion ${LMI_LIB_FILES})
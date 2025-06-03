cmake_minimum_required(VERSION 3.5)
project(eroam)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g ${CMAKE_CXX_FLAGS}")

add_definitions("-DCATKIN_ENABLE_TESTING=0")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# tbb
find_package(TBB REQUIRED)

# eigen 3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/ikd-Tree)

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# ros
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        velodyne_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        dv_ros_msgs       
        dv_ros_messaging  
        rosbag
)
include_directories(${catkin_INCLUDE_DIRS})

# include roscpp
include_directories(${roscpp_INCLUDE_DIRS})

set(third_party_libs
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${Pangolin_LIBRARIES}
        glog gflags
        ${yaml-cpp_LIBRARIES}
        yaml-cpp
        TBB::tbb
        ${Python3_LIBRARIES}
)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/ikd-Tree)

add_executable(${PROJECT_NAME}.eroam_run
        src/eroam_run.cpp
        thirdparty/ikd-Tree/ikd_Tree.cpp
        src/timer.cc
)

target_link_libraries(${PROJECT_NAME}.eroam_run
        ${third_party_libs}
)

add_executable(${PROJECT_NAME}.rosbag_repack
        src/rosbag_repack.cpp
)

target_link_libraries(${PROJECT_NAME}.rosbag_repack
        ${third_party_libs}
)

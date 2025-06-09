//
// Created by wlxing.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include "eroam.hpp"

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "run_eroam");
    ros::NodeHandle nh;

    eroam::EROAM eroam;
    eroam.Init(nh);

    ros::spin();

    LOG(INFO) << "done.";

    return 0;
}

/**
 * @file
 * @brief Test Path Optimizer.
 */

#include <string>
#include <glog/logging.h>
#include <ros/package.h>
#include <ros/ros.h>
#include "config/planning_flags.h"

using namespace mujianhua::planning;

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_path_optimizer");

    std::string base_dir = ros::package::getPath("test_planning");
    auto log_dir = base_dir + "/log";

    google::InitGoogleLogging(argv[0]);

    FLAGS_log_dir = log_dir;

    LOG(INFO) << "car_width: " << FLAGS_car_width;

    return 0;
}
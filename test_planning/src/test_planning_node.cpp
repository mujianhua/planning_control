#include "test_common/PathPoint.h"
#include "test_common/TrajectoryPoint.h"
#include "gtest/gtest.h"
#include <ros/ros.h>

class PlanningNode {
  public:
    PlanningNode(ros::NodeHandle nh) : nh_(nh) {
        planning_pub_ =
            nh_.advertise<test_common::TrajectoryPoint>("planning", 1);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher planning_pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_planning");
    ros::NodeHandle nh;

    return 0;
}
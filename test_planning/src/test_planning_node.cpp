#include <ros/ros.h>
#include "gtest/gtest.h"
#include "test_common/PathPoint.h"
#include "test_common/TrajectoryPoint.h"

class PlanningNode {
  public:
    explicit PlanningNode(const ros::NodeHandle &nh) : nh_(nh) {
        planning_pub_ =
            nh_.advertise<test_common::TrajectoryPoint>("planning", 1);
        test_common::TrajectoryPoint trajectory_point;
        trajectory_point.path_point.x = 0.1;
        planning_pub_.publish(trajectory_point);
        ROS_INFO("%f", trajectory_point.path_point.x);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher planning_pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_planning");
    ros::NodeHandle nh;

    PlanningNode node(nh);

    ros::spin();

    return 0;
}
#include <ros/ros.h>
#include "common/math/polygon2d.h"

class TestPlanningNode {
  public:
  private:
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_planning_node");
    ros::NodeHandle nh;

    ROS_INFO("test planning node!");
    ros::spin();
}
#include "planning_node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_planning_node");
    ros::NodeHandle nh;
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
    //                                ros::console::levels::Debug);
    mujianhua::planning::visualization::Init(nh, "map",
                                             "cartesian_planner_markers");

    mujianhua::planning::PlanningNode node(nh);

    ros::spin();
}

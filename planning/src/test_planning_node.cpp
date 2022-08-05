#include <cstdint>
#include <memory>
#include <ros/ros.h>
#include "common/dependency_injector.h"
#include "common/math/polygon2d.h"
#include "on_lane_planning.h"
#include "planning_base.h"
#include "ros/node_handle.h"

using namespace mujianhua::planning;

class TestPlanningNode {
  public:
    TestPlanningNode(const ros::NodeHandle &nh) : nh_(nh) {
        injector_ = std::make_shared<common::DependencyInjector>();
        planning_ = std::make_shared<OnLanePlanning>(injector_);
        planning_->Init();
        planning_->RunOnce();
    }

  private:
    ros::NodeHandle nh_;
    std::shared_ptr<PlanningBase> planning_;
    std::shared_ptr<common::DependencyInjector> injector_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_planning_node");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Debug);
    TestPlanningNode node(nh);

    ros::spin();
}
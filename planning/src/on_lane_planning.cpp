#include "on_lane_planning.h"
#include <memory>
#include <ros/ros.h>
#include "planner/cartesian_planner.h"
#include "reference_line/reference_line_provider.h"

namespace mujianhua {
namespace planning {

bool OnLanePlanning::Init() {
    reference_line_provider_ = std::make_unique<ReferenceLineProvider>();
    planner_ = std::make_unique<CartesianPlanner>(injector_);

    ROS_INFO("planner is %s", planner_->Name().c_str());
    return true;
}

void OnLanePlanning::RunOnce() { ROS_DEBUG("NOW begin to plan!"); }

} // namespace planning
} // namespace mujianhua
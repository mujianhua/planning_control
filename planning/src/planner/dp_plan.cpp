#include "planner/dp_plan.h"
#include "config/planning_config.h"
#include "ros/ros.h"

namespace mujianhua {
namespace planning {

DPPlan::DPPlan(const PlanningConfig &config) : config_(config) {}

bool DPPlan::Plan(const common::State &start_state, const common::Frame *frame,
                  common::DiscretizedTrajectory &result) {
    ROS_DEBUG("[DP Plan]");
    frame_ = frame;
    auto sl =
        frame_->reference_line()->GetProjection({start_state.x, start_state.y});

    return true;
}

} // namespace planning
} // namespace mujianhua
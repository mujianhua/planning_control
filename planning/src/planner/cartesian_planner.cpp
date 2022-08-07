#include "planner/cartesian_planner.h"
#include <string>
#include "planner/dp_plan.h"
#include "planner/planner.h"

namespace mujianhua {
namespace planning {

CartesianPlanner::CartesianPlanner(
    const std::shared_ptr<common::DependencyInjector> &injector)
    : Planner(injector) {}

bool CartesianPlanner::Init(const PlanningConfig &config) {
    dp_ = new DPPlan(config);
    config_ = config;
    return true;
}

bool CartesianPlanner::Plan(
    const common::State &start_state, common::Frame *frame,
    common::DiscretizedTrajectory *ptr_computed_trajectory) {

    common::DiscretizedTrajectory coarse_trajectory;
    dp_->Plan(start_state, frame, coarse_trajectory);

    return true;
}

} // namespace planning
} // namespace mujianhua
#include "planner/cartesian_planner.h"
#include <string>
#include "planner/planner.h"

namespace mujianhua {
namespace planning {

CartesianPlanner::CartesianPlanner(
    const std::shared_ptr<common::DependencyInjector> &injector)
    : Planner(injector) {}

bool CartesianPlanner::Plan(const State &start_state, common::Frame *frame) {
    return true;
}

} // namespace planning
} // namespace mujianhua
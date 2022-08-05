#include "planner/cartesian_planner.h"
#include <string>
#include "planner/planner.h"

namespace mujianhua {
namespace planning {

CartesianPlanner::CartesianPlanner(
    const std::shared_ptr<common::DependencyInjector> &injector)
    : Planner(injector) {}

} // namespace planning
} // namespace mujianhua
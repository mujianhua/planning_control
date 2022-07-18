/**
 * @file
 */

#include "test_planning/lattice_planner.h"
#include "test_planning/planner.h"

namespace mujianhua {
namespace planning {

common::Status LatticePlanner::Init(const PlanningConfig &planning_config) {
    return common::Status::OK();
}

bool LatticePlanner::Plan(
    const common_me::TrajectoryPoint &planning_init_point) {
    return true;
}

} // namespace planning
} // namespace mujianhua
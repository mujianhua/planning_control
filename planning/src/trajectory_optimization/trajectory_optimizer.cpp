#include "trajectory_optimization/trajectory_optimizer.h"
#include "config/planning_config.h"

namespace mujianhua {
namespace planning {

TrajectoryOptimizer::TrajectoryOptimizer(const PlanningConfig &config,
                                         const Frame *frame)
    : config_(config), frame_(frame) {}

bool TrajectoryOptimizer::OptimizeIteratively(
    const DiscretizedTrajectory &coarse, const OptiConstraints &constraints,
    OptiStates &result) {
    return true;
}

} // namespace planning
} // namespace mujianhua

#pragma once

#include "common/discretized_trajectory.h"
#include "common/frame.h"
#include "config/planning_config.h"
#include "trajectory_optimization/trajectory_nlp_optimizer.h"

namespace mujianhua {
namespace planning {

class TrajectoryOptimizer {
  public:
    TrajectoryOptimizer(const PlanningConfig &config, const Frame *frame);

    bool OptimizeIteratively(const DiscretizedTrajectory &coarse,
                             const OptiConstraints &constraints,
                             OptiStates &result);

  private:
    const PlanningConfig config_;
    const Frame *frame_;
};

} // namespace planning
} // namespace mujianhua

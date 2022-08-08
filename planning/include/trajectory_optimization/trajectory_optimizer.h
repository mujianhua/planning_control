#pragma once

#include "common/discretized_trajectory.h"
#include "common/frame.h"
#include "config/planning_config.h"
#include "constraint_checker/collision_checker.h"
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
    TrajectoryNLPOptimizer nlp_;
    CollisionChecker *collision_checker_;

    void CalculateInitialGuess(OptiStates &states) const;

    bool FormulateCorridorConstraints(OptiStates &states,
                                      OptiConstraints &constraints);

    bool GenerateBox(double time, double &x, double &y, double radius,
                     math::AABox2d &result) const;

    bool CheckCollision(double time, double x, double y,
                        const math::AABox2d &bound) const;
};

} // namespace planning
} // namespace mujianhua

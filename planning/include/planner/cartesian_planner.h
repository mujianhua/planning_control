#pragma once

#include <memory>
#include <string>
#include "planner/dp_plan.h"
#include "planner/planner.h"
#include "trajectory_optimization/trajectory_optimizer.h"

namespace mujianhua {
namespace planning {

class CartesianPlanner : public Planner {
  public:
    CartesianPlanner() = delete;

    explicit CartesianPlanner(
        const std::shared_ptr<DependencyInjector> &injector);

    bool Init(const PlanningConfig &config) override;

    std::string Name() override { return "Cartesian Planner"; };

    bool Plan(const State &start_state, Frame *frame,
              DiscretizedTrajectory *ptr_computed_trajectory) override;

  private:
    DPPlan *dp_;
    TrajectoryOptimizer *opti_;
};

} // namespace planning
} // namespace mujianhua
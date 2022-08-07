#pragma once

#include <memory>
#include <string>
#include "planner/dp_plan.h"
#include "planner/planner.h"

namespace mujianhua {
namespace planning {

class CartesianPlanner : public Planner {
  public:
    CartesianPlanner() = delete;

    explicit CartesianPlanner(
        const std::shared_ptr<common::DependencyInjector> &injector);

    bool Init(const PlanningConfig &config) override;

    std::string Name() override { return "Cartesian Planner"; };

    bool Plan(const common::State &start_state, common::Frame *frame,
              common::DiscretizedTrajectory *ptr_computed_trajectory) override;

  private:
    DPPlan *dp_;
};

} // namespace planning
} // namespace mujianhua
#pragma once

#include "planner/planner.h"
#include "planning/planning_config.h"

namespace planning {

class LatticePlanner : public Planner {
  public:
    explicit LatticePlanner(const PlanningConfig &config) : Planner(config) {}

    bool Plan(const StartState &state, const std::shared_ptr<Frame> &frame,
              DiscretizedTrajectory &result) override;

    std::string Name() override { return "Lattice"; }

  private:
};

} // namespace planning

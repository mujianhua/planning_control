#pragma once

#include <string>

#include "planning/discretized_trajectory.h"
#include "planning/frame.h"
#include "planning/planning_config.h"

namespace planning {

class Planner {
 public:
  Planner() = default;

  explicit Planner(const PlanningConfig &config) : config_(config) {}

  virtual bool Plan(const VehicleState &state, Frame *frame,
                    DiscretizedTrajectory &result) = 0;

  virtual std::string Name() = 0;

 protected:
  PlanningConfig config_;
};

}  // namespace planning

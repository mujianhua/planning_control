
#pragma once

#include <string>

#include "planning/data_struct.h"
#include "planning/discretized_trajectory.h"
#include "planning/frame.h"
#include "planning/planning_config.h"

namespace planning {

class Planner {
 public:
  Planner() = delete;

  explicit Planner(const PlanningConfig &config) : config_(config) {}

  virtual bool Plan(const TrajectoryPoint &planning_init_point, Frame *frame,
                    DiscretizedTrajectory &result) = 0;

  virtual std::string Name() = 0;

 protected:
  PlanningConfig config_;
};

}  // namespace planning

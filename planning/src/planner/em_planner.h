#pragma once

#include "planner/planner.h"

namespace planning {

class EMPlanner : public Planner {
 public:
  explicit EMPlanner(const PlanningConfig &config);

  bool Plan(const TrajectoryPoint &planning_init_point, Frame *frame,
            DiscretizedTrajectory &result) override;

  std::string Name() override { return "EM Planner"; }

 private:
};

}  // namespace planning

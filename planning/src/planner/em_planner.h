#pragma once

#include "planner/planner.h"

namespace planning {

class EMPlanner : public Planner {
 public:
  explicit EMPlanner(const PlanningConfig &config) : Planner(config) {}

  std::string Name() override { return "EM Planner"; }

 private:
};

}  // namespace planning

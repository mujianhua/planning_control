#pragma once

#include <array>
#include "planner/planner.h"
#include "planning/data_struct.h"
#include "planning/planning_config.h"

namespace planning {

class LatticePlanner : public Planner {
 public:
  explicit LatticePlanner(const PlanningConfig &config) : Planner(config) {}

  bool Plan(const VehicleState &state, Frame *frame,
            DiscretizedTrajectory &result) override;

  std::string Name() override { return "Lattice"; }

 private:
  void ComputeInitFrenetState(const TrajectoryPoint &match_point,
                              const TrajectoryPoint &cartesian_point,
                              std::array<double, 3> *ptr_s,
                              std::array<double, 3> *ptr_d);
};

}  // namespace planning

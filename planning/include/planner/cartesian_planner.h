/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source
 *codes. Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#pragma once

#include "planning/planning_config.h"

#include "planner/dp_planner.h"
#include "planner/planner.h"
#include "planning/trajectory_optimizer.h"

namespace planning {

class CartesianPlanner : public Planner {
 public:
  explicit CartesianPlanner(const PlanningConfig &config)
      : Planner(config_), dp_(config), opti_(config) {}

  bool Plan(const TrajectoryPoint &planning_init_point, Frame *frame,
            DiscretizedTrajectory &result) override;

  std::string Name() override { return "Cartesian Planner"; }

 private:
  PlanningConfig config_;
  DpPlanner dp_;
  TrajectoryOptimizer opti_;
};

}  // namespace planning

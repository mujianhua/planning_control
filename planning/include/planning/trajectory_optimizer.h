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

#include "trajectory_nlp.h"

#include "math/aabox2d.h"
#include "math/polygon2d.h"

#include "discretized_trajectory.h"
#include "planning_config.h"

namespace planning {

using math::AABox2d;
using math::Box2d;
using math::Polygon2d;

class TrajectoryOptimizer {
 public:
  TrajectoryOptimizer(const PlanningConfig &config);

  bool OptimizeIteratively(const DiscretizedTrajectory &coarse, Frame *frame,
                           const Constraints &constraints, States &result);

 private:
  PlanningConfig config_;
  Frame *frame_;
  VehicleParam vehicle_;
  TrajectoryNLP nlp_;

  void CalculateInitialGuess(States &states) const;

  bool FormulateCorridorConstraints(States &states, Constraints &constraints);

  bool GenerateBox(double time, double &x, double &y, double radius,
                   AABox2d &result) const;

  inline bool CheckCollision(double time, double x, double y,
                             const AABox2d &bound) const {
    Box2d box(bound);
    box.Shift({x, y});

    return frame_->CheckCollision(time, box);
  }
};

}  // namespace planning

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

#include "../math/aabox2d.h"
#include "../math/polygon2d.h"
#include "common/planning_config.h"
#include "discretized_trajectory.h"
#include "trajectory_nlp.h"

namespace planning {

using math::AABox2d;
using math::Box2d;
using math::Polygon2d;

class TrajectoryOptimizer {
 public:
  explicit TrajectoryOptimizer(const PlanningConfig &config);

  bool OptimizeIteratively(const DiscretizedTrajectory &coarse, Frame *frame,
                           const OptiConstraints &constraints,
                           OptiStates &result);

 private:
  PlanningConfig config_;
  Frame *frame_{};
  VehicleParam vehicle_;
  TrajectoryNLP nlp_;

  /**
   * @brief calculate other OptiStates by dp coarse trajectory.
   */
  void CalculateInitialGuess(OptiStates &states) const;

  bool FormulateCorridorConstraints(OptiStates &states,
                                    OptiConstraints &constraints);

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

/**
 * @file cartesian_planner.cpp
 */

#include "planner/cartesian_planner.h"

#include "common/data_struct.h"
#include "planner/dp_planner.h"
#include "visualization/plot.h"

namespace planning {

namespace {

void set_opti_constraints(OptiConstraints &opti_constraints,
                          const Frame *frame) {
  opti_constraints.start_x = frame->vehicle_state().x;
  opti_constraints.start_y = frame->vehicle_state().y;
  opti_constraints.start_theta = frame->vehicle_state().theta;
  opti_constraints.start_v = frame->vehicle_state().v;
  opti_constraints.start_phi = frame->vehicle_state().phi;
  opti_constraints.start_a = frame->vehicle_state().a;
  opti_constraints.start_omega = frame->vehicle_state().omega;
}

}  // namespace

bool CartesianPlanner::Plan(const TrajectoryPoint &planning_init_point,
                            Frame *frame, DiscretizedTrajectory &result) {
  ros::Time start_time = ros::Time::now();
  ros::Time current_time = start_time;

  /** 1. Get coarse trajectory by dynamic programming */
  DiscretizedTrajectory coarse_trajectory;
  if (!dp_.Plan(planning_init_point.x, planning_init_point.y,
                planning_init_point.theta, frame, coarse_trajectory)) {
    ROS_ERROR("DP failed");
    return false;
  }

  ROS_DEBUG("[CartesianPlanner] dp time is %f",
            (ros::Time::now() - current_time).toSec());
  current_time = ros::Time::now();

  /*  // visualize coarse trajectory.
   std::vector<double> coarse_x, coarse_y;
   for (auto &pt : coarse_trajectory.data()) {
     coarse_x.push_back(pt.x);
     coarse_y.push_back(pt.y);
   }
   visualization::Plot(coarse_x, coarse_y, 0.1, visualization::Color::Cyan, 1,
                       "Coarse Trajectory");
   visualization::PlotPoints(coarse_x, coarse_y, 0.3,
   visualization::Color::Cyan,
                             2, "Coarse Trajectory");
   visualization::Trigger();
 */

  /** 2. optimize coarse trajectory. */
  OptiConstraints opti_constraints;
  set_opti_constraints(opti_constraints, frame);

  OptiStates optimized;
  if (!opti_.OptimizeIteratively(coarse_trajectory, frame, opti_constraints,
                                 optimized)) {
    ROS_ERROR("Optimization failed");
    return false;
  }

  ROS_DEBUG("[CartesianPlanner] iterative optimization time is %f",
            (ros::Time::now() - current_time).toSec());
  current_time = ros::Time::now();

  std::vector<double> opti_x, opti_y, opti_v;
  Trajectory result_data;
  double incremental_s = 0.0;
  for (int i = 0; i < config_.nfe; i++) {
    incremental_s += i > 0 ? hypot(optimized.x[i] - optimized.x[i - 1],
                                   optimized.y[i] - optimized.y[i - 1])
                           : 0.0;
    TrajectoryPoint tp(incremental_s, optimized.x[i], optimized.y[i],
                       optimized.theta[i],
                       tan(optimized.phi[i]) / config_.vehicle.wheel_base);
    tp.velocity = optimized.v[i];

    opti_x.push_back(tp.x);
    opti_y.push_back(tp.y);
    opti_v.push_back(tp.velocity);

    result_data.push_back(tp);
  }

  visualization::PlotTrajectory(
      opti_x, opti_y, opti_v, config_.vehicle.max_velocity, 0.1,
      visualization::Color::Green, 1, "Optimized Trajectory");
  visualization::Trigger();

  result = DiscretizedTrajectory(result_data);
  return true;
}

}  // namespace planning

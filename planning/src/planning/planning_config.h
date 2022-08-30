
#pragma once

#include <gflags/gflags.h>
#include <gflags/gflags_declare.h>

#include "vehicle_param.h"

namespace planning {

DECLARE_string(planner);

DECLARE_bool(enable_reference_line_provider_thread);

DECLARE_bool(enable_smooth_reference_line);

////////////////////////////////////////////////////
//                  LATTICE                       //
////////////////////////////////////////////////////
DECLARE_double(trajectory_time_length);

struct PlanningConfig {
  /**
   * @brief Number of finite elements used to discretize an OCP
   */
  int nfe = 320;

  /**
   * @brief Time horizon length (s)
   */
  double tf = 16;

  /**
   * @brief nominal velocity
   */
  double dp_nominal_velocity = 10.0;

  /**
   * @brief cost of obstacles, should be set larger to avoid collision with
   * obstacles
   */
  double dp_w_obstacle = 1000;

  /**
   * @brief lateral cost, the larger the trajectory the closer to the
   * reference line
   */
  double dp_w_lateral = 0.1;

  /**
   * @brief lateral change cost, dl/ds, penalty for lateral change
   */
  double dp_w_lateral_change = 0.5;

  /**
   * @brief lateral change cost, dl/dt, penalty for sudden lateral change
   */
  double dp_w_lateral_velocity_change = 1.0;

  /**
   * @brief longitudinal velocity cost, velocity to the nominal velocity
   */
  double dp_w_longitudinal_velocity_bias = 10.0;

  /**
   * @brief Cost of longitudinal velocity change, ds/dt
   */
  double dp_w_longitudinal_velocity_change = 1.0;

  /**
   * @brief maximum iteration count for corridor expansion
   */
  int corridor_max_iter = 1000;

  /**
   * @brief increment limit for corridor expansion
   */
  double corridor_incremental_limit = 20.0;

  /**
   * @brief Weighting parameter in Eq.(2)
   */
  double opti_w_u = 0.5;

  /**
   * @brief weighting parameter in Eq.(3)
   */
  double opti_w_r_theta = 2.0;

  /**
   * @brief weighting parameter in Eq.(4)
   */
  double opti_w_rw = 5.0;

  /**
   * @brief Maximum iteration number in Alg.1
   */
  int opti_iter_max = 5;

  /**
   * @brief Initial value of weighting parameter w_penalty
   */
  double opti_w_penalty0 = 1e5;

  /**
   * @brief Multiplier to enlarge w_penalty during the iterations
   */
  double opti_alpha = 10;

  /**
   * @brief Violation tolerance w.r.t. the softened nonlinear constraints
   */
  double opti_varepsilon_tol = 1e-4;

  VehicleParam vehicle;
};

}  // namespace planning

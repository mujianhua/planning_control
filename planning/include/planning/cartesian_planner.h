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

#include "planning_config.h"

#include "dp_planner.h"
#include "trajectory_optimizer.h"

namespace planning {

class CartesianPlanner {
  public:
    struct StartState {
        StartState(double xx = 0.0, double yy = 0.0, double ttheta = 0.0,
                   double vv = 0.0, double pphi = 0.0, double aa = 0.0,
                   double oomega = 0.0)
            : x(xx), y(yy), theta(ttheta), v(vv), phi(pphi), a(aa),
              omega(oomega) {}
        double x, y, theta, v, phi, a, omega;
    };

    explicit CartesianPlanner(const CartesianPlannerConfig &config,
                              const std::shared_ptr<Frame> &frame)
        : config_(config), dp_(config, frame), opti_(config, frame) {}

    bool Plan(const StartState &state, DiscretizedTrajectory &result);

  private:
    CartesianPlannerConfig config_;
    DpPlanner dp_;
    TrajectoryOptimizer opti_;
};

} // namespace planning
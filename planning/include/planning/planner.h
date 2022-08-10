#pragma once

#include "planning/discretized_trajectory.h"
#include "planning/frame.h"
#include "planning/planning_config.h"

namespace planning {

class Planner {
  public:
    struct StartState {
        StartState(double xx = 0.0, double yy = 0.0, double ttheta = 0.0,
                   double vv = 0.0, double pphi = 0.0, double aa = 0.0,
                   double oomega = 0.0)
            : x(xx), y(yy), theta(ttheta), v(vv), phi(pphi), a(aa),
              omega(oomega) {}
        double x, y, theta, v, phi, a, omega;
    };

    Planner() = default;

    explicit Planner(const PlanningConfig &config) : config_(config) {}

    virtual bool Plan(const StartState &state,
                      const std::shared_ptr<Frame> &frame,
                      DiscretizedTrajectory &result) = 0;

  protected:
    PlanningConfig config_;
};

} // namespace planning

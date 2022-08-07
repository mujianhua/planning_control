#pragma once

#include <casadi/casadi.hpp>

#include "common/discretized_trajectory.h"
#include "config/planning_config.h"

namespace mujianhua {
namespace planning {

using namespace casadi;

struct OptiStates {
    std::vector<double> x, y, theta, v, phi, a, omega, jerk, xf, yf, xr, yr;
};

struct OptiConstraints {
    OptiConstraints(double x = 0.0, double y = 0.0, double theta = 0.0,
                    double v = 0.0, double a = 0.0, double phi = 0.0,
                    double omega = 0.0) {}
    double start_x, start_y, start_theta, start_v, start_a, start_phi,
        start_omega;
    std::vector<std::array<double, 4>> front_bound;
    std::vector<std::array<double, 4>> rear_bound;
};

class TrajectoryNLPOptimizer {
  public:
    explicit TrajectoryNLPOptimizer(const PlanningConfig &);

    double SolveIteratively(double w_inf, const OptiConstraints &constraints,
                            const OptiStates &guess,
                            const DiscretizedTrajectory &reference,
                            OptiStates &result);

  private:
    PlanningConfig config_;
    Dict nlp_config_;
    Function iterative_solver_;
    Function infeasibility_evaluator_;

    void BuildIterativeNLP();
};

} // namespace planning
} // namespace mujianhua

#pragma once

#include <vector>

#include "common/planning_gflags.h"

namespace planning {

class LateralQPOptimizer {
 public:
  LateralQPOptimizer() = default;

  virtual ~LateralQPOptimizer() = default;

  virtual bool optimize(
      const std::array<double, 3> &d_state, const double delta_s,
      const std::vector<std::pair<double, double>> &d_bounds) = 0;

 protected:
  double delta_s_ = FLAGS_default_delta_s_lateral_optimization;

  std::vector<double> opt_d_;
  std::vector<double> opt_d_prime_;
  std::vector<double> opt_d_pprime_;
};

}  // namespace planning

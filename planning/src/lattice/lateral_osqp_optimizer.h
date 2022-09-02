#pragma once

#include <array>
#include <vector>

#include "lattice/lateral_qp_optimizer.h"
#include "osqp/osqp.h"

namespace planning {

class LateralOSQPOptimizer : public LateralQPOptimizer {
 public:
  LateralOSQPOptimizer() = default;

  virtual ~LateralOSQPOptimizer() = default;

  bool optimize(
      const std::array<double, 3> &d_state, const double delta_s,
      const std::vector<std::pair<double, double>> &d_bounds) override;

 private:
  void CalculateKernel(const std::vector<std::pair<double, double>> &d_bounds,
                       std::vector<c_float> *P_data,
                       std::vector<c_int> *P_indices,
                       std::vector<c_int> *P_indptr);
};

}  // namespace planning

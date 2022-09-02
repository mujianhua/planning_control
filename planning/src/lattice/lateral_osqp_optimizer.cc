#include "lattice/lateral_osqp_optimizer.h"

namespace planning {

bool LateralOSQPOptimizer::optimize(
    const std::array<double, 3>& d_state, const double delta_s,
    const std::vector<std::pair<double, double>>& d_bounds) {}

void LateralOSQPOptimizer::CalculateKernel(
    const std::vector<std::pair<double, double>>& d_bounds,
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr) {
  const int kNumParam = 3 * static_cast<int>(d_bounds.size());
}

}  // namespace planning

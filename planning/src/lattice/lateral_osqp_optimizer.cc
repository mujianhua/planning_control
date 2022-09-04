#include "lattice/lateral_osqp_optimizer.h"

#include <vector>
#include "glog/logging.h"

#include "common/planning_gflags.h"

namespace planning {

bool LateralOSQPOptimizer::optimize(
    const std::array<double, 3>& d_state, const double delta_s,
    const std::vector<std::pair<double, double>>& d_bounds) {
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(d_bounds, &P_data, &P_indices, &P_indptr);
  delta_s_ = delta_s;
  const int num_var = static_cast<int>(d_bounds.size());
  const int kNumParam = 3 * num_var;
  const int kNumConstraint = kNumParam + 3 * (num_var - 1) + 3;
  c_float lower_bounds[kNumConstraint];
  c_float upper_bounds[kNumConstraint];

  const int prime_offest = num_var;
  const int pprime_offest = 2 * num_var;

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam);

  int constraint_index = 0;

  // (d_i+1)'' - (d_i)''
  for (int i = 0; i + 1 < num_var; ++i) {
    columns[pprime_offest + i].emplace_back(constraint_index, -1.0);
    columns[pprime_offest + i + 1].emplace_back(constraint_index, 1.0);

    lower_bounds[constraint_index] =
        -FLAGS_weight_lateral_third_order_derivative * delta_s_;
    upper_bounds[constraint_index] =
        FLAGS_weight_lateral_third_order_derivative * delta_s_;
    ++constraint_index;
  }

  // ((d_i+1)' - (d_i)')/ds = 0.5*(d_i'' + d_i+1'')
  for (int i = 0; i + 1 < num_var; ++i) {
    columns[prime_offest + i].emplace_back(constraint_index, -1.0);
    columns[prime_offest + i + 1].emplace_back(constraint_index, 1.0);
    columns[pprime_offest + i].emplace_back(constraint_index, -0.5 * delta_s_);
    columns[pprime_offest + i + 1].emplace_back(constraint_index,
                                                -0.5 * delta_s_);

    lower_bounds[constraint_index] = 0.0;
    upper_bounds[constraint_index] = 0.0;
    ++constraint_index;
  }

  // d_i+1 = d_i + d_i' * d_s + 1/3 * d_i'' * d_s^2 + 1/6 * d_i+1'' * d_s^2
  for (int i = 0; i + 1 < num_var; ++i) {
    columns[i].emplace_back(constraint_index, -1.0);
    columns[i + 1].emplace_back(constraint_index, 1.0);
    columns[prime_offest + i].emplace_back(constraint_index, -delta_s_);
    columns[pprime_offest + i].emplace_back(constraint_index,
                                            -delta_s_ * delta_s_ / 3.0);
    columns[pprime_offest + i + 1].emplace_back(constraint_index,
                                                -delta_s_ * delta_s_ / 6.0);

    lower_bounds[constraint_index] = 0.0;
    upper_bounds[constraint_index] = 0.0;
    ++constraint_index;
  }
}

void LateralOSQPOptimizer::CalculateKernel(
    const std::vector<std::pair<double, double>>& d_bounds,
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr) {
  const int kNumParam = 3 * static_cast<int>(d_bounds.size());
  P_data->resize(kNumParam);
  P_indices->reserve(kNumParam);
  P_indptr->reserve(kNumParam + 1);

  for (int i = 0; i != kNumParam; ++i) {
    if (i < static_cast<int>(d_bounds.size())) {
      P_data->at(i) = 2.0 * FLAGS_weight_lateral_offset +
                      2.0 * FLAGS_weight_lateral_obstacle_distance;
    } else if (i < 2 * static_cast<int>(d_bounds.size())) {
      P_data->at(i) = 2.0 * FLAGS_weight_lateral_derivative;
    } else {
      P_data->at(i) = 2 * FLAGS_weight_lateral_second_order_derivative;
    }
    P_indices->at(i) = i;
    P_indptr->at(i) = i;
  }
  P_indptr->at(kNumParam) = kNumParam;
  CHECK_EQ(P_data->size(), P_indices->size());
}

}  // namespace planning

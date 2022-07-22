#pragma once

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include "common_me/TrajectoryPoint.h"
#include "data_struct/reference_path.h"
#include "tools/vehicle_state.h"

namespace mujianhua {
namespace planning {

using common_me::TrajectoryPoint;

class BaseSolver {
  public:
    BaseSolver() = delete;
    BaseSolver(ReferencePath *reference_path, VehicleState *vehicle_state,
               int iter_num, bool enable_hard_constraint);

    bool Solve(std::vector<TrajectoryPoint> *optimized_path);

  private:
    void SetHessianMatrix(Eigen::SparseMatrix<double> *matrix_h);

    const int iter_num_{};
    bool enable_hard_constraint_{};
    const size_t n_{};
    size_t state_size_{};
    size_t control_size_{};
    size_t slack_size_{};
    size_t vars_size_{};
    size_t cons_size_{};
    ReferencePath *reference_path_;
    VehicleState *vehicle_state_;
    OsqpEigen::Solver solver_;
};

} // namespace planning
} // namespace mujianhua
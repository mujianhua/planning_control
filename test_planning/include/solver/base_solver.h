#pragma once

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>
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
    void SetHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const;

    void SetConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                             Eigen::VectorXd *lower_bound,
                             Eigen::VectorXd *upper_bound) const;

    std::pair<double, double> GetSoftBounds(double lb, double ub,
                                            double safety_margin) const;

    void GetOptimizedPath(const Eigen::VectorXd &optimization_result,
                          std::vector<TrajectoryPoint> *optimized_path) const;

    const int iter_num_{};
    bool enable_hard_constraint_{};
    const int n_{};
    int state_size_{};   // [e_y e_psi, kappa]
    int control_size_{}; // [d_kappa]
    int slack_size_{};
    int vars_size_{};
    int cons_size_{};
    ReferencePath *reference_path_;
    VehicleState *vehicle_state_;
    OsqpEigen::Solver solver_;
};

} // namespace planning
} // namespace mujianhua
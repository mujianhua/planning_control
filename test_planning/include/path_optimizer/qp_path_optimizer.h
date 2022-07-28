#pragma once

#include <cstddef>
#include "common/frame.h"
#include "data_struct/data_struct.h"
#include "path_optimizer/path_optimizer.h"
#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

class QPPathOptimizer {
  public:
    QPPathOptimizer() = delete;

    QPPathOptimizer(ReferenceLine *reference_line, Frame *frame,
                    bool enable_hard_constraint);

    bool Solve(std::vector<PathPoint> *optimized_path);

  private:
    void SetHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) const;

    void SetConstraintMatrix(Eigen::SparseMatrix<double> *matrix_constraints,
                             Eigen::VectorXd *lower_bound,
                             Eigen::VectorXd *upper_bound) const;

    std::pair<double, double> GetSoftBounds(double lb, double ub,
                                            double safety_margin) const;

    void GetOptimizedPath(const Eigen::VectorXd &optimization_result,
                          std::vector<PathPoint> *optimized_path) const;

    bool enable_hard_constraint_{};
    const size_t n_{};
    int state_size_{};   // [e_y e_psi, kappa]
    int control_size_{}; // [d_kappa]
    int slack_size_{};
    int vars_size_{};
    int cons_size_{};
    ReferenceLine *reference_line_;
    Frame *frame_;
    OsqpEigen::Solver solver_;
};

} // namespace planning
} // namespace mujianhua
#include "solver/base_solver.h"

namespace mujianhua {
namespace planning {

mujianhua::planning::BaseSolver::BaseSolver(
    ReferencePath *reference_path,
    mujianhua::planning::VehicleState *vehicle_state, int iter_num,
    bool enable_hard_constraint)
    : n_(reference_path->GetSize()), iter_num_(iter_num),
      enable_hard_constraint_(enable_hard_constraint),
      reference_path_(reference_path), vehicle_state_(vehicle_state) {
    state_size_ = 3 * n_;
    control_size_ = n_ - 1;
    slack_size_ = 2 * n_;
    vars_size_ = state_size_ + control_size_ + slack_size_;
    cons_size_ =
        6 * n_ + 2 + 2 * n_ * static_cast<int>(enable_hard_constraint_);
}

bool BaseSolver::Solve(std::vector<TrajectoryPoint> *optimized_path) {
    const auto &ref_points = reference_path_->GetReferencePoints();
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.data()->setNumberOfVariables(vars_size_);
    solver_.data()->setNumberOfConstraints(cons_size_);
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(vars_size_);
    Eigen::SparseMatrix<double> linear_matrix;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;

    return true;
}

void BaseSolver::SetHessianMatrix(Eigen::SparseMatrix<double> *matrix_h) {
    static const auto weight_l = 0.1;
    static const auto weight_kappa = 20.0;
    static const auto weight_d_kappa = 100.0;
    static const auto weight_slack = 10; // 1000.0 - 200 * iter_num_;

    Eigen::MatrixXd hessian{
        Eigen::MatrixXd::Constant(vars_size_, vars_size_, 0.0)};
}

} // namespace planning
} // namespace mujianhua

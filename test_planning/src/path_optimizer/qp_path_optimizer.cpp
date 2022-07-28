#include "path_optimizer/qp_path_optimizer.h"
#include "common/data_struct.h"
#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

QPPathOptimizer::QPPathOptimizer(const ReferenceLine *reference_line,
                                 const Frame *frame,
                                 bool enable_hard_constraint)
    : n_(reference_line->size()),
      enable_hard_constraint_(enable_hard_constraint),
      reference_line_(reference_line), frame_(frame) {
    state_size_ = 3 * n_;
    control_size_ = n_ - 1;
    slack_size_ = 2 * n_;
    vars_size_ = state_size_ + control_size_ + slack_size_;
    cons_size_ =
        6 * n_ + 2 + 2 * n_ * static_cast<int>(enable_hard_constraint_);
}

bool QPPathOptimizer::Solve(std::vector<PathPoint> *optimized_path) {
    const auto &ref_points = reference_line_->reference_points();
    solver_.settings()->setVerbosity(false);
    solver_.settings()->setWarmStart(true);
    solver_.data()->setNumberOfVariables(vars_size_);
    solver_.data()->setNumberOfConstraints(cons_size_);

    Eigen::SparseMatrix<double> hessian;
    SetHessianMatrix(&hessian);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(vars_size_);
    Eigen::SparseMatrix<double> linear_matrix;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;
    SetConstraintMatrix(&linear_matrix, &lower_bound, &upper_bound);
    if (!solver_.data()->setHessianMatrix(hessian))
        return false;
    if (!solver_.data()->setGradient(gradient))
        return false;
    if (!solver_.data()->setLinearConstraintsMatrix(linear_matrix))
        return false;
    if (!solver_.data()->setLowerBound(lower_bound))
        return false;
    if (!solver_.data()->setUpperBound(upper_bound))
        return false;
    if (!solver_.initSolver())
        return false;
    auto status = solver_.solveProblem();
    const auto &qpsolution = solver_.getSolution();
    GetOptimizedPath(qpsolution, optimized_path);
    return true;
}

void QPPathOptimizer::SetHessianMatrix(
    Eigen::SparseMatrix<double> *matrix_h) const {
    static constexpr auto weight_l = 0.1;
    static constexpr auto weight_kappa = 20.0;
    static constexpr auto weight_d_kappa = 100.0;
    static constexpr auto weight_slack = 10.0; // 1000.0 - 200 * iter_num_;

    Eigen::MatrixXd hessian{
        Eigen::MatrixXd::Constant(vars_size_, vars_size_, 0.0)};
    for (int i = 0; i < n_; i++) {
        hessian(3 * i, 3 * i) += weight_l;
        hessian(3 * i + 2, 3 * i + 2) += weight_kappa;
        if (i != n_ - 1)
            hessian(state_size_ + i, state_size_ + i) += weight_d_kappa;
        hessian(state_size_ + control_size_ + 2 * i,
                state_size_ + control_size_ + 2 * i) += weight_slack;
        hessian(state_size_ + control_size_ + 2 * i + 1,
                state_size_ + control_size_ + 2 * i + 1) += weight_slack;
    }
    *matrix_h = hessian.sparseView();
}

void QPPathOptimizer::SetConstraintMatrix(
    Eigen::SparseMatrix<double> *matrix_constraints,
    Eigen::VectorXd *lower_bound, Eigen::VectorXd *upper_bound) const {
    const auto &ref_points = reference_line_->reference_points();
    const int trans_idx = 0;
    const int kappa_idx = trans_idx + 3 * n_;
    const int collision_idx = kappa_idx + n_;
    const int end_state_idx = collision_idx + 2 * n_;
    const int hard_constraint_idx = end_state_idx + 2;
    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(cons_size_, vars_size_);
    // transition function
    for (int i = 0; i < state_size_; i++) {
        cons(i, i) = -1;
    }
    Eigen::Matrix3d a(Eigen::Matrix3d::Zero());
    a(0, 1) = 1;
    a(1, 2) = 1;
    Eigen::Matrix<double, 3, 1> b(Eigen::MatrixXd::Constant(3, 1, 0));
    b(2, 0) = 1;
    std::vector<Eigen::MatrixXd> c_list;
    for (int i = 0; i < n_ - 1; i++) {
        const auto ref_k = ref_points[i].kappa;
        const auto ds = ref_points[i + 1].s - ref_points[i].s;
        const auto ref_d_k = (ref_points[i + 1].kappa - ref_k) / ds;
        a(1, 0) = -pow(ref_k, 2);
        auto A = a * ds + Eigen::Matrix3d::Identity();
        auto B = b * ds;
        cons.block(3 * (i + 1), 3 * i, 3, 3) = A;
        cons.block(3 * (i + 1), state_size_ + i, 3, 1) = B;
        Eigen::Matrix<double, 3, 1> c, ref_state;
        c << 0, 0, ref_d_k;
        ref_state << 0, 0, ref_k;
        c_list.emplace_back(ds * (c - a * ref_state - b * ref_d_k));
    }
    // kappa
    for (int i = 0; i < n_; i++) {
        cons(kappa_idx + i, 3 * i + 2) = 1;
    }
    // collision
    Eigen::Matrix<double, 2, 2> collision_coeff;
    collision_coeff << 1, FLAGS_front_length, 1, FLAGS_rear_length;
    Eigen::Matrix<double, 2, 2> slack_coeff;
    slack_coeff << 1, 0, 0, 1;
    for (int i = 0; i < n_; i++) {
        cons.block(collision_idx + 2 * i, 3 * i, 2, 2) = collision_coeff;
        cons.block(collision_idx + 2 * i, state_size_ + control_size_ + 2 * i,
                   2, 2) = slack_coeff;
    }
    // end state
    cons(end_state_idx, state_size_ - 3) = 1;     // l
    cons(end_state_idx + 1, state_size_ - 2) = 1; // e_psi
    // hard constraint
    if (enable_hard_constraint_) {
        for (int i = 0; i < n_; i++) {
            cons.block(hard_constraint_idx + 2 * i, 3 * i, 2, 2) =
                collision_coeff;
        }
    }
    *matrix_constraints = cons.sparseView();

    *lower_bound = Eigen::VectorXd::Zero(cons_size_, 1);
    *upper_bound = Eigen::VectorXd::Zero(cons_size_, 1);
    // transition function
    Eigen::Matrix<double, 3, 1> x0;
    const auto init_lateral_error =
        frame_->GetVehicleStartState()->lateral_error();
    const auto init_heading_error =
        frame_->GetVehicleStartState()->heading_error();

    // TODO: kappa
    x0 << init_lateral_error, init_heading_error, 0.0;

    lower_bound->block(0, 0, 3, 1) = -x0;
    upper_bound->block(0, 0, 3, 1) = -x0;
    for (int i = 0; i < n_ - 1; i++) {
        lower_bound->block(3 * (i + 1), 0, 3, 1) = -c_list[i];
        upper_bound->block(3 * (i + 1), 0, 3, 1) = -c_list[i];
    }
    // kappa
    for (int i = 0; i < n_; i++) {
        (*lower_bound)(kappa_idx + i) =
            -tan(FLAGS_max_steering_angle) / FLAGS_wheel_base;
        (*upper_bound)(kappa_idx + i) =
            tan(FLAGS_max_steering_angle) / FLAGS_wheel_base;
    }
    // collision
    const auto &bounds = reference_line_->GetBounds();
    for (int i = 0; i < n_; i++) {
        const auto front_bound =
            GetSoftBounds(bounds[i].front.lb, bounds[i].front.ub,
                          FLAGS_expected_safety_margin);
        const auto rear_bound = GetSoftBounds(
            bounds[i].rear.lb, bounds[i].rear.ub, FLAGS_expected_safety_margin);
        (*lower_bound)(collision_idx + 2 * i, 0) = front_bound.first;
        (*upper_bound)(collision_idx + 2 * i, 0) = front_bound.second;
        (*lower_bound)(collision_idx + 2 * i + 1, 0) = rear_bound.first;
        (*upper_bound)(collision_idx + 2 * i + 1, 0) = rear_bound.second;
    }
    // end state
    (*lower_bound)(end_state_idx) = -1.0; // l
    (*upper_bound)(end_state_idx) = 1.0;
    (*lower_bound)(end_state_idx + 1) = -OsqpEigen::INFTY; // e_psi
    (*upper_bound)(end_state_idx + 1) = OsqpEigen::INFTY;
    if (FLAGS_constraint_end_heading && reference_line_->IsBlocked()) {
        double end_psi =
            math::ConstrainAngle(frame_->GetVehicleTargetState()->heading() -
                                 ref_points.back().theta);
        if (end_psi < 70 * M_PI / 180) {
            (*lower_bound)(end_state_idx + 1) = end_psi - 0.087; // e_psi
            (*upper_bound)(end_state_idx + 1) = end_psi + 0.087;
        }
    }
    if (enable_hard_constraint_) {
        for (int i = 0; i < n_; i++) {
            (*lower_bound)(hard_constraint_idx + 2 * i, 0) = bounds[i].front.lb;
            (*upper_bound)(hard_constraint_idx + 2 * i, 0) = bounds[i].front.ub;
            (*lower_bound)(hard_constraint_idx + 2 * i + 1, 0) =
                bounds[i].rear.lb;
            (*upper_bound)(hard_constraint_idx + 2 * i + 1, 0) =
                bounds[i].rear.ub;
        }
    }
}

std::pair<double, double>
QPPathOptimizer::GetSoftBounds(double lb, double ub,
                               double safety_margin) const {
    static const auto min_clearance = 0.1;
    const auto clearance = ub - lb;
    auto remain_clearance =
        std::max(min_clearance, clearance - 2 * safety_margin);
    auto shrink = std::max(0.0, (clearance - remain_clearance) / 2.0);
    return std::make_pair(lb + shrink, ub - shrink);
}

void QPPathOptimizer::GetOptimizedPath(
    const Eigen::VectorXd &optimization_result,
    std::vector<PathPoint> *optimized_path) const {

    CHECK_EQ(vars_size_, optimization_result.size());
    optimized_path->clear();
    const auto &ref_points = reference_line_->reference_points();
    double tmp_s = 0.0;
    for (int i = 0; i < n_; i++) {
        PathPoint tmp_point;
        ROS_DEBUG("l: %f", optimization_result(3 * i));
        double angle = ref_points[i].theta;
        tmp_point.x =
            ref_points[i].x + optimization_result(3 * i) *
                                  cos(math::ConstrainAngle(angle + M_PI_2));
        tmp_point.y =
            ref_points[i].y + optimization_result(3 * i) *
                                  sin(math::ConstrainAngle(angle + M_PI_2));
        tmp_point.kappa = optimization_result(3 * i + 2);
        tmp_point.theta =
            math::ConstrainAngle(angle + optimization_result(3 * i + 1));
        if (i != 0) {
            tmp_s += sqrt(pow(tmp_point.x - optimized_path->back().x, 2) +
                          pow(tmp_point.y - optimized_path->back().y, 2));
        }

        tmp_point.s = tmp_s;
        optimized_path->emplace_back(tmp_point);
    }
}

} // namespace planning
} // namespace mujianhua
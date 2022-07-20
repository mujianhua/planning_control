#include "path_smoother/tension_smoother.h"

namespace mujianhua {
namespace planning {

TensionSmoother::TensionSmoother(
    const std::vector<TrajectoryPoint> &initial_points,
    const TrajectoryPoint &start_point, const Map &grid_map)
    : ReferencePathSmoother(initial_points, start_point, grid_map) {}

bool TensionSmoother::Smooth(ReferencePath *reference_path) {
    ROS_INFO("This is Tension Smoother smooth method.");
    std::vector<double> x_list, y_list, s_list, heading_list, kappa_list;
    if (!SegmentRawReference(&x_list, &y_list, &s_list, &heading_list,
                             &kappa_list)) {
        ROS_ERROR("Unable segment raw reference path.");
        return false;
    }
    std::vector<double> result_x_list, result_y_list, result_s_list;
    OsqpSmooth(x_list, y_list, s_list, heading_list, kappa_list, &result_x_list,
               &result_y_list, &result_s_list);

    result_x_list = x_list;
    result_y_list = y_list;
    result_s_list = s_list;

    tk::spline x_spline, y_spline;
    x_spline.set_points(result_s_list, result_x_list);
    y_spline.set_points(result_s_list, result_y_list);
    double max_result_s = result_s_list.back() + 3.0;
    reference_path->SetSpline(x_spline, y_spline, max_result_s);

    x_list_ = result_x_list;
    y_list_ = result_y_list;
    s_list_ = result_s_list;
    return true;
}

bool TensionSmoother::OsqpSmooth(
    const std::vector<double> &x_list, const std::vector<double> &y_list,
    const std::vector<double> &s_list, const std::vector<double> &heading_list,
    const std::vector<double> &kappa_list, std::vector<double> *result_x_list,
    std::vector<double> *result_y_list, std::vector<double> *result_s_list) {
    CHECK_EQ(x_list.size(), y_list.size());
    CHECK_EQ(y_list.size(), s_list.size());
    CHECK_EQ(s_list.size(), heading_list.size());

    auto point_num = x_list.size();
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(point_num);
    solver.data()->setNumberOfConstraints(point_num);
    Eigen::SparseMatrix<double> hessian;
    OsqpSetHessianMatrix(point_num, &hessian);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(3 * point_num);
    Eigen::SparseMatrix<double> linear_matrix;
    Eigen::VectorXd lowerbound;
    Eigen::VectorXd upperbound;
    OsqpSetConstraintMatrix(x_list, y_list, s_list, heading_list, kappa_list,
                            &linear_matrix, &lowerbound, &upperbound);

    return true;
}

void TensionSmoother::OsqpSetHessianMatrix(
    size_t size, Eigen::SparseMatrix<double> *matrix_h) const {}

void TensionSmoother::OsqpSetConstraintMatrix(
    const std::vector<double> &x_list, const std::vector<double> &y_list,
    const std::vector<double> &s_list, const std::vector<double> &heading_list,
    const std::vector<double> &kappa_list,
    Eigen::SparseMatrix<double> *matrix_constraints,
    Eigen::VectorXd *lowerbound, Eigen::VectorXd *upperbound) const {}

bool TensionSmoother::IpoptSmooth(
    const std::vector<double> &x_list, const std::vector<double> &y_list,
    const std::vector<double> &s_list, const std::vector<double> &heading_list,
    const std::vector<double> &kappa_list, std::vector<double> *result_x_list,
    std::vector<double> *result_y_list, std::vector<double> *result_s_list) {

    CHECK_EQ(x_list.size(), y_list.size());
    CHECK_EQ(y_list.size(), s_list.size());
    CHECK_EQ(s_list.size(), heading_list.size());

    size_t n_vars = x_list.size();
    CPPAD_TESTVECTOR(double) vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    // bounds of variables
    CPPAD_TESTVECTOR(double) vars_lowerbound(n_vars);
    CPPAD_TESTVECTOR(double) vars_upperbound(n_vars);
    vars_lowerbound[0] = 0.0;
    vars_upperbound[0] = 0.0;
    vars_lowerbound[n_vars - 1] = -0.5;
    vars_upperbound[n_vars - 1] = 0.5;

    static const double default_clearance{1};
    for (size_t i = 1; i < n_vars - 1; i++) {
        double clearance = grid_map_.GetObstacleDistance(
            grid_map::Position(x_list[i], y_list[i]));
        clearance = std::max(default_clearance, clearance);
        vars_lowerbound[i] = -clearance;
        vars_upperbound[i] = clearance;
    }

    std::string options;
    CppAD::ipopt::solve_result<CPPAD_TESTVECTOR(double)> solution;

    return true;
}

} // namespace planning
} // namespace mujianhua

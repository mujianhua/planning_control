#include "path_smoother/reference_path_smoother.h"
#include <math.h>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <vector>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/SparseCore/SparseMatrix.h>
#include <OsqpEigen/Solver.hpp>
#include <glog/logging.h>
#include "common_me/TrajectoryPoint.h"
#include "config/planning_flags.h"
#include "grid_map_core/TypeDefs.hpp"
#include "math/math_util.h"
#include "path_smoother/tension_smoother.h"
#include "tools/spline.h"

namespace mujianhua {
namespace planning {

ReferencePathSmoother::ReferencePathSmoother(
    const std::vector<TrajectoryPoint> &initial_points,
    const TrajectoryPoint &start_point, const Map &grid_map)
    : initial_points_(initial_points), start_point_(start_point),
      grid_map_(grid_map) {}

std::unique_ptr<ReferencePathSmoother> ReferencePathSmoother::Creat(
    std::string &type, const std::vector<TrajectoryPoint> &initial_points,
    const TrajectoryPoint &start_point, const Map &grid_map) {
    return std::unique_ptr<ReferencePathSmoother>{
        new TensionSmoother(initial_points, start_point, grid_map)};
}

bool ReferencePathSmoother::Solve(ReferencePath *reference_path) {
    if (initial_points_.size() < 4) {
        LOG(ERROR) << "The reference points is too few.";
    }
    bSpline();

    if (!Smooth(reference_path)) {
        ROS_ERROR("unable smooth reference path");
    }

    if (!GraphSearchDp(reference_path)) {
        ROS_ERROR("unable smooth graph search");
    }

    if (!PostSmooth(reference_path)) {
        ROS_ERROR("unable post smooth");
    }

    return true;
}

void ReferencePathSmoother::bSpline() {
    double length = 0.0;
    for (size_t i = 0; i < initial_points_.size() - 1; ++i) {
        length += math::Distance(initial_points_[i], initial_points_[i + 1]);
    }
    double average_length = length / (initial_points_.size() - 1);
    int degree = 3;
    if (average_length > 10)
        degree = 3;
    else if (average_length > 5)
        degree = 4;
    else
        degree = 5;
    tinyspline::BSpline b_spline_raw(initial_points_.size(), 2, degree);
    std::vector<tinyspline::real> ctrl_point_raw = b_spline_raw.controlPoints();
    for (size_t i = 0; i < initial_points_.size(); i++) {
        ctrl_point_raw[2 * i] = initial_points_[i].path_point.x;
        ctrl_point_raw[2 * i + 1] = initial_points_[i].path_point.y;
    }
    b_spline_raw.setControlPoints(ctrl_point_raw);
    double delta_t = 1.0 / length;
    double tmp_t = 0.0;
    while (tmp_t < 1.0) {
        auto result = b_spline_raw.eval(tmp_t).result();
        x_list_.emplace_back(result[0]);
        y_list_.emplace_back(result[1]);
        tmp_t += delta_t;
    }
    auto result = b_spline_raw.eval(1).result();
    x_list_.emplace_back(result[0]);
    y_list_.emplace_back(result[1]);
    s_list_.emplace_back(0);
    for (size_t i = 1; i < x_list_.size(); ++i) {
        double distance = sqrt(pow(x_list_[i] - x_list_[i - 1], 2) +
                               pow(y_list_[i] - y_list_[i - 1], 2));
        s_list_.emplace_back(s_list_.back() + distance);
    }
}

bool ReferencePathSmoother::SegmentRawReference(
    std::vector<double> *x_list, std::vector<double> *y_list,
    std::vector<double> *s_list, std::vector<double> *heading_list,
    std::vector<double> *kappa_list) const {
    if ((x_list_.size() != s_list_.size()) ||
        (y_list_.size() != s_list_.size())) {
        LOG(ERROR) << "Raw Path x y and s is not equal.";
        return false;
    }
    tk::spline x_spline, y_spline;
    x_spline.set_points(s_list_, x_list_);
    y_spline.set_points(s_list_, y_list_);
    double max_s = s_list_.back();
    double delta_s = 1.0;
    s_list->emplace_back(0);
    while (s_list->back() < max_s) {
        s_list->emplace_back(s_list->back() + delta_s);
    }
    for (double length_on_ref_path : *s_list) {
        double dx = x_spline.deriv(1, length_on_ref_path);
        double dy = y_spline.deriv(1, length_on_ref_path);
        double ddx = x_spline.deriv(2, length_on_ref_path);
        double ddy = y_spline.deriv(2, length_on_ref_path);
        double theta = atan2(dy, dx);
        double curvature = (dx * ddy - dy * ddx) / pow(dx * dx + dy * dy, 1.5);
        heading_list->emplace_back(theta);
        kappa_list->emplace_back(curvature);
        x_list->emplace_back(x_spline(length_on_ref_path));
        y_list->emplace_back(y_spline(length_on_ref_path));
    }
    return true;
}

bool ReferencePathSmoother::GraphSearchDp(ReferencePath *referemce_path) {
    static const double search_threshold = fLD::FLAGS_car_width / 2.0 + 0.2;
    const tk::spline &x_s = referemce_path->GetXS();
    const tk::spline &y_s = referemce_path->GetYS();
    layers_s_list_.clear();
    layers_bounds_.clear();
    double search_ds = referemce_path->GetLength() > 6.0
                           ? FLAGS_search_longitudial_spacing
                           : 0.5;
    TrajectoryPoint start_prj_point = math::GetProjectPoint(
        x_s, y_s, start_point_.path_point.x, start_point_.path_point.y,
        referemce_path->GetLength());

    double tmp_s = start_prj_point.path_point.s;
    while (tmp_s < referemce_path->GetLength()) {
        layers_s_list_.emplace_back(tmp_s);
        tmp_s += search_ds;
    }
    layers_s_list_.emplace_back(referemce_path->GetLength());
    if (layers_s_list_.empty())
        return false;
    target_s_ = layers_s_list_.back();
    auto vehicle_local = math::Global2Local(start_prj_point, start_point_);
    if (fabs(vehicle_local.path_point.y) > FLAGS_search_lateral_range) {
        LOG(ERROR) << "Vehicle start point far from reference path, quit graph "
                      "searching";
        return false;
    }
    vehicle_l_wrt_smoothed_ref_ = vehicle_local.path_point.y;
    int start_lateral_index = static_cast<int>(
        (FLAGS_search_lateral_range + vehicle_local.path_point.y) /
        FLAGS_search_lateral_spacing);
    std::vector<std::vector<DPPoint>> samples;
    samples.reserve(layers_s_list_.size());

    for (int i = 0; i < layers_s_list_.size(); ++i) {
        samples.emplace_back(std::vector<DPPoint>());
        double cur_s = layers_s_list_[i];
        double ref_x = x_s(cur_s);
        double ref_y = y_s(cur_s);
        double ref_heading = math::GetHeading(x_s, y_s, cur_s);
        double ref_curvature = math::GetCurvature(x_s, y_s, cur_s);
        double ref_r = 1.0 / ref_curvature;
        int lateral_index = 0;
        double cur_l = -FLAGS_search_lateral_range;
        while (cur_l <= FLAGS_search_lateral_range) {
            DPPoint dp_point;
            dp_point.x = ref_x + cur_l * cos(ref_heading + M_PI_2);
            dp_point.y = ref_y + cur_l * sin(ref_heading + M_PI_2);
            dp_point.heading = ref_heading;
            dp_point.s = cur_s;
            dp_point.l = cur_l;
            dp_point.layer_index = i;
            dp_point.lateral_index = lateral_index;
            grid_map::Position node_pos(dp_point.x, dp_point.y);
            dp_point.dis_to_obs = grid_map_.IsInside(node_pos)
                                      ? grid_map_.GetObstacleDistance(node_pos)
                                      : -1;
            if ((ref_curvature < 0 && cur_l < ref_r) ||
                (ref_curvature > 0 && cur_l > ref_r) ||
                dp_point.dis_to_obs < search_threshold) {
                dp_point.is_feasible = false;
            }
            if (i == 0 && start_lateral_index != lateral_index) {
                dp_point.is_feasible = false;
            }
            if (i == 0 && start_lateral_index == lateral_index) {
                dp_point.is_feasible = true;
                dp_point.dir = start_point_.path_point.theta;
                dp_point.cost = 0.0;
            }
            samples.back().emplace_back(dp_point);
            ++lateral_index;
            cur_l += FLAGS_search_lateral_spacing;
        }
        // get layer point set rough bound.
        auto &layer_point_set = samples.back();
        for (size_t j = 0; j < layer_point_set.size(); j++) {
            if (j == 0 || !layer_point_set[j - 1].is_feasible ||
                !layer_point_set[j].is_feasible) {
                layer_point_set[j].rough_lower_bound = layer_point_set[j].l;
            } else {
                layer_point_set[j].rough_lower_bound =
                    layer_point_set[j - 1].rough_lower_bound;
            }
        }
        for (int j = layer_point_set.size() - 1; j >= 0; j--) {
            if (j == layer_point_set.size() - 1 ||
                !layer_point_set[j + 1].is_feasible ||
                !layer_point_set[j].is_feasible) {
                layer_point_set[j].rough_upper_bound = layer_point_set[j].l;
            } else {
                layer_point_set[j].rough_upper_bound =
                    layer_point_set[j + 1].rough_upper_bound;
            }
        }
    }
    // calculate cost
    int max_layer_reached = 0;
    for (const auto &layer : samples) {
        bool is_layer_feasible = false;
        for (const auto &point : layer) {
            CalculateCost(samples, point.layer_index, point.lateral_index);
            if (point.parent)
                is_layer_feasible = true;
        }
        if (layer.front().layer_index != 0 && !is_layer_feasible)
            break;
        max_layer_reached = layer.front().layer_index;
    }
    // retrieve path.
    const DPPoint *ptr = nullptr; // 指向的地址的内容不能变，指向的地址是可变的
    auto min_cost = DBL_MAX;
    for (const auto &point : samples[max_layer_reached]) {
        if (point.cost < min_cost) {
            ptr = &point;
            min_cost = point.cost;
        }
    }
    while (ptr) {
        // 准备后续优化路径的边界
        if (ptr->lateral_index == 0) {
            layers_bounds_.emplace_back(-10, 10);
        } else {
            static const double check_s = 0.2;
            static const double check_limit = 6.0;
            double upper_bound = check_s + ptr->rough_upper_bound;
            double lower_bound = -check_s + ptr->rough_lower_bound;
            double ref_x = x_s(ptr->s);
            double ref_y = y_s(ptr->s);
            while (upper_bound < check_limit) {
                grid_map::Position pos;
                pos(0) = ref_x + upper_bound * cos(ptr->heading + M_PI_2);
                pos(1) = ref_y + upper_bound * sin(ptr->heading + M_PI_2);
                if (grid_map_.IsInside(pos) &&
                    grid_map_.GetObstacleDistance(pos) > search_threshold) {
                    upper_bound += check_s;
                } else {
                    upper_bound -= check_s;
                    break;
                }
            }
            while (lower_bound > -check_limit) {
                grid_map::Position pos;
                pos(0) = ref_x + lower_bound * cos(ptr->heading + M_PI_2);
                pos(1) = ref_y + lower_bound * sin(ptr->heading + M_PI_2);
                if (grid_map_.IsInside(pos) &&
                    grid_map_.GetObstacleDistance(pos) > search_threshold) {
                    lower_bound -= check_s;
                } else {
                    lower_bound += check_s;
                    break;
                }
            }
            layers_bounds_.emplace_back(lower_bound, upper_bound);
        }
        ptr = ptr->parent;
    }
    std::reverse(layers_bounds_.begin(), layers_bounds_.end());
    layers_s_list_.resize(layers_bounds_.size());
    return true;
}

bool ReferencePathSmoother::PostSmooth(ReferencePath *reference_path) {
    auto point_num = layers_s_list_.size();
    if (point_num < 4) {
        LOG(WARNING) << "ref is too short:" << point_num
                     << ", quit POST SMOOTHING.";
    }

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(3 * point_num);
    solver.data()->setNumberOfConstraints(3 * point_num - 2);

    Eigen::SparseMatrix<double> hessian;
    SetPostHessianMatrix(&hessian);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(3 * point_num);
    Eigen::SparseMatrix<double> linear_matrix;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;
    SetPostConstraintMatrix(&linear_matrix, &lower_bound, &upper_bound);

    if (!solver.data()->setHessianMatrix(hessian))
        return false;
    if (!solver.data()->setGradient(gradient))
        return false;
    if (!solver.data()->setLinearConstraintsMatrix(linear_matrix))
        return false;
    if (!solver.data()->setLowerBound(lower_bound))
        return false;
    if (!solver.data()->setUpperBound(upper_bound))
        return false;

    if (!solver.initSolver())
        return false;
    auto status = solver.solveProblem();
    const auto &qpsolution = solver.getSolution();

    std::vector<double> x_list, y_list, s_list;
    const auto &ref_xs = reference_path->GetXS();
    const auto &ref_ys = reference_path->GetYS();
    double s = 0;
    for (int i = 0; i < point_num; i++) {
        const double ref_s = layers_s_list_[i];
        double ref_dir = math::GetHeading(ref_xs, ref_ys, ref_s);
        x_list.push_back(ref_xs(ref_s) + qpsolution(i) * cos(ref_dir + M_PI_2));
        y_list.push_back(ref_ys(ref_s) + qpsolution(i) * sin(ref_dir + M_PI_2));
        if (i > 0) {
            s += sqrt(pow(x_list[i] - x_list[i - 1], 2) +
                      pow(y_list[i] - y_list[i - 1], 2));
        }
        s_list.push_back(s);
    }
    tk::spline new_x_s, new_y_s;
    new_x_s.set_points(s_list, x_list);
    new_y_s.set_points(s_list, y_list);
    reference_path->SetSpline(new_x_s, new_y_s, s_list.back());

    return true;
}

void ReferencePathSmoother::SetPostHessianMatrix(
    Eigen::SparseMatrix<double> *matrix_h) {
    size_t size = layers_s_list_.size();
    const size_t matrix_size = 3 * size;
    Eigen::MatrixXd hessian =
        Eigen::MatrixXd::Constant(matrix_size, matrix_size, 0);
    static const double weight_l = 1;
    static const double weight_dl = 100;
    static const double weight_ddl = 1000;
    for (int i = 0; i < size; ++i) {
        hessian(i, i) = weight_l;
        hessian(size + i, size + i) = weight_dl;
        hessian(size * 2 + i, size * 2 + i) = weight_ddl;
    }
    *matrix_h = hessian.sparseView();
}

void ReferencePathSmoother::SetPostConstraintMatrix(
    Eigen::SparseMatrix<double> *matrix_constraints,
    Eigen::VectorXd *lower_bound, Eigen::VectorXd *upper_bound) const {
    size_t size = layers_s_list_.size();
    const size_t x_index = 0;
    const size_t dx_index = x_index + size;
    const size_t ddx_index = dx_index + size;
    const size_t cons_x_index = 0;
    const size_t cons_dx_x_index = cons_x_index + size;
    const size_t cons_ddx_dx_index = cons_dx_x_index + size - 1;
    Eigen::MatrixXd cons = Eigen::MatrixXd::Zero(3 * size - 2, 3 * size);
    // l range
    for (int i = 0; i < size; ++i) {
        cons(i, i) = 1;
    }
    // dl range
    for (int i = 0; i < size - 1; ++i) {
        cons(cons_dx_x_index + i, x_index + i) = -1;
        cons(cons_dx_x_index + i, x_index + i + 1) = 1;
        cons(cons_dx_x_index + i, dx_index + i) =
            -(layers_s_list_[i + 1] - layers_s_list_[i]);
    }
    // ddl range
    for (int i = 0; i < size - 1; ++i) {
        cons(cons_ddx_dx_index + i, dx_index + i) = -1;
        cons(cons_ddx_dx_index + i, dx_index + i + 1) = 1;
        cons(cons_ddx_dx_index + i, ddx_index + i) =
            -(layers_s_list_[i + 1] - layers_s_list_[i]);
    }
    *matrix_constraints = cons.sparseView();
    // bounds
    *lower_bound = Eigen::VectorXd::Zero(3 * size - 2);
    *upper_bound = Eigen::VectorXd::Zero(3 * size - 2);
    (*lower_bound)(0) = vehicle_l_wrt_smoothed_ref_;
    (*upper_bound)(0) = vehicle_l_wrt_smoothed_ref_;
    for (int i = 1; i < size; ++i) {
        (*lower_bound)(i) = layers_bounds_[i].first;
        (*upper_bound)(i) = layers_bounds_[i].second;
    }
}

void ReferencePathSmoother::CalculateCost(
    std::vector<std::vector<DPPoint>> &samples, int layer_index,
    int lateral_index) {
    // TODO: 加载外部参数调节
    static const double weight_ref_offest = 1.0;
    static const double weight_obstacle = 0.5;
    static const double weight_angle_change = 16.0;
    static const double weight_ref_angle_diff = 0.5;
    static const double safe_distance = 3.0;
    if (layer_index == 0)
        return;
    if (!samples[layer_index][lateral_index].is_feasible)
        return;
    auto &point = samples[layer_index][lateral_index];
    double self_cost = 0.0;
    if (point.dis_to_obs < safe_distance) {
        self_cost += (safe_distance - point.dis_to_obs) / safe_distance *
                     weight_obstacle;
    }
    self_cost +=
        fabs(point.l) / fLD::FLAGS_search_lateral_range * weight_ref_offest;

    auto min_cost = DBL_MAX;
    for (const auto &pre_point : samples[layer_index - 1]) {
        if (!pre_point.is_feasible)
            continue;
        if (fabs(pre_point.l - point.l) > (point.s - pre_point.s))
            continue;
        double direction = atan2(point.y - pre_point.y, point.x - pre_point.x);
        double edge_cost =
            fabs(math::ConstrainAngle(direction - pre_point.dir)) / M_PI_2 *
                weight_angle_change +
            fabs(math::ConstrainAngle(direction - point.heading)) / M_PI_2 *
                weight_ref_angle_diff;
        double total_cost = self_cost + edge_cost + pre_point.cost;
        if (total_cost < min_cost) {
            min_cost = total_cost;
            point.parent = &pre_point;
            point.dir = direction;
        }
    }
    if (point.parent)
        point.cost = min_cost;
}

} // namespace planning
} // namespace mujianhua

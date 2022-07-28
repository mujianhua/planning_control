#include "reference_line/qp_spline_reference_smoother.h"
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>
#include "data_struct/data_struct.h"
#include "reference_line/reference_line.h"
#include "reference_line/reference_point.h"

namespace mujianhua {
namespace planning {

namespace {

double Distance(const ReferencePoint &p1, const ReferencePoint &p2) {
    return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2));
}

double Distance(const PathPoint &p1, const PathPoint &p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

struct DP_POINT {
    double x{}, y{}, heading{}, s{}, l{}, dir{}, dis_to_obs{};
    double cost = DBL_MAX;
    double rough_upper_bound{}, rough_lower_bound{};
    int layer_index{}, lateral_index{};
    const DP_POINT *parent = nullptr;
    bool is_feasible = true;
};

} // namespace

QPSplineReferenceLineSmoother::QPSplineReferenceLineSmoother()
    : name_("QPSpline") {}

QPSplineReferenceLineSmoother::QPSplineReferenceLineSmoother(
    std::shared_ptr<PlanningDependencyInjector> injector)
    : injector_(std::move(injector)), name_("QPSpline") {}

const std::string QPSplineReferenceLineSmoother::Name() const { return name_; }

bool QPSplineReferenceLineSmoother::Smooth(
    const std::vector<PathPoint> &raw_reference_points,
    ReferenceLine *smoothed_reference_line, Frame *frame) {
    if (raw_reference_points.size() < 5) {
        ROS_ERROR("[ReferenceLineSmoother] the initial reference points is too "
                  "little!");
        return false;
    }
    ref_points_ = raw_reference_points;

    if (!SplineInterpolation()) {
        ROS_ERROR("[ReferenceLineSmoother] unable spline interpolate for raw "
                  "reference points!");
    }

    std::vector<double> x_list, y_list, s_list, theta_list, kappa_list;
    if (!SegmetRawReferencePoints(&x_list, &y_list, &s_list, &theta_list,
                                  &kappa_list)) {
        ROS_ERROR(
            "[ReferenceLineSmoother] unable segment raw reference points!");
    }

    tk::spline x_spline, y_spline;
    x_spline.set_points(s_list, x_list);
    y_spline.set_points(s_list, y_list);
    double max_result_s = s_list.back() + 3.0;
    smoothed_reference_line->SetSpline(x_spline, y_spline, max_result_s);

    if (!DPGraphSearch(frame, smoothed_reference_line)) {
        ROS_ERROR("unable add bounds by dp search");
    }

    if (!Smooth(smoothed_reference_line)) {
        ROS_ERROR("...");
    }

    return true;
}

bool QPSplineReferenceLineSmoother::SplineInterpolation() {
    double length = 0.0;
    for (size_t i = 0; i < ref_points_.size() - 1; ++i) {
        length += Distance(ref_points_[i], ref_points_[i + 1]);
    }
    double average_length = length / (ref_points_.size() - 1);
    int degree = 3;
    if (average_length > 10)
        degree = 3;
    else if (average_length > 5)
        degree = 4;
    else
        degree = 5;
    tinyspline::BSpline b_spline_raw(ref_points_.size(), 2, degree);
    std::vector<tinyspline::real> ctrl_point_raw = b_spline_raw.controlPoints();
    for (size_t i = 0; i < ref_points_.size(); i++) {
        ctrl_point_raw[2 * i] = ref_points_[i].x;
        ctrl_point_raw[2 * i + 1] = ref_points_[i].y;
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
    return true;
}

bool QPSplineReferenceLineSmoother::SegmetRawReferencePoints(
    std::vector<double> *x_list, std::vector<double> *y_list,
    std::vector<double> *s_list, std::vector<double> *theta_list,
    std::vector<double> *kappa_list) {
    if ((x_list_.size() != s_list_.size()) ||
        (y_list_.size() != s_list_.size())) {
        ROS_ERROR("Raw Path x y and s is not equal!");
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
        theta_list->emplace_back(theta);
        kappa_list->emplace_back(curvature);
        x_list->emplace_back(x_spline(length_on_ref_path));
        y_list->emplace_back(y_spline(length_on_ref_path));
    }
    return true;
}

bool QPSplineReferenceLineSmoother::DPGraphSearch(
    Frame *frame, ReferenceLine *reference_line) {
    static const double search_threshold = FLAGS_car_width / 2.0 + 0.2;
    dp_layers_s_list_.clear();
    layers_bounds_.clear();
    double search_ds = reference_line->GetLength() > 6.0
                           ? FLAGS_search_longitudial_spacing
                           : 0.5;

    ReferencePoint start_prj_point = reference_line->FindProjectPoint(
        frame->GetVehicleStartState()->x(), frame->GetVehicleStartState()->y());

    double tmp_s = start_prj_point.s();
    while (tmp_s < reference_line->GetLength()) {
        // 以车辆投影点为起点开始搜索平滑！
        dp_layers_s_list_.emplace_back(tmp_s);
        tmp_s += search_ds;
    }

    dp_layers_s_list_.emplace_back(reference_line->GetLength());
    if (dp_layers_s_list_.empty())
        return false;
    auto target_s = dp_layers_s_list_.back();
    auto vehicle_local =
        math::Global2Local(start_prj_point, *frame->GetVehicleStartState());
    if (fabs(vehicle_local.y()) > FLAGS_search_lateral_range) {
        ROS_ERROR("Vehicle start point far from reference path, quit graph "
                  "searching");
        return false;
    }
    vehicle_l_wrt_smoothed_ref_ = vehicle_local.y();
    int start_lateral_index =
        static_cast<int>((FLAGS_search_lateral_range + vehicle_local.y()) /
                         FLAGS_search_lateral_spacing);
    std::vector<std::vector<DP_POINT>> samples;
    samples.reserve(dp_layers_s_list_.size());

    for (int i = 0; i < dp_layers_s_list_.size(); ++i) {
        samples.emplace_back(std::vector<DP_POINT>());

        double cur_s = dp_layers_s_list_[i];
        ReferencePoint ref_point = reference_line->GetRerencePoint(cur_s);

        int lateral_index = 0;
        double cur_l = -FLAGS_search_lateral_range;
        while (cur_l <= FLAGS_search_lateral_range) {
            DP_POINT dp_point;
            dp_point.x =
                ref_point.x() + cur_l * cos(ref_point.theta() + M_PI_2);
            dp_point.y =
                ref_point.y() + cur_l * sin(ref_point.theta() + M_PI_2);
            dp_point.heading = ref_point.theta();
            dp_point.s = cur_s;
            dp_point.l = cur_l;
            dp_point.layer_index = i;
            dp_point.lateral_index = lateral_index;
            grid_map::Position node_pos(dp_point.x, dp_point.y);
            dp_point.dis_to_obs =
                frame->GridMap()->IsInside(node_pos)
                    ? frame->GridMap()->GetObstacleDistance(node_pos)
                    : -1;
            if ((ref_point.kappa() < 0 && cur_l < 1.0 / ref_point.kappa()) ||
                (ref_point.kappa() > 0 && cur_l > 1.0 / ref_point.kappa()) ||
                dp_point.dis_to_obs < search_threshold) {
                dp_point.is_feasible = false;
            }
            if (i == 0 && start_lateral_index != lateral_index) {
                dp_point.is_feasible = false;
            }
            if (i == 0 && start_lateral_index == lateral_index) {
                dp_point.is_feasible = true;
                dp_point.dir = frame->GetVehicleStartState()->heading();
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
            DPCalculateCost(samples, point.layer_index, point.lateral_index);
            if (point.parent)
                is_layer_feasible = true;
        }
        if (layer.front().layer_index != 0 && !is_layer_feasible)
            break;
        max_layer_reached = layer.front().layer_index;
    }
    // retrieve path.
    const DP_POINT *ptr = nullptr; // 指向的地址的内容不能变，指向的地址是可变的
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
            ReferencePoint ref_point = reference_line->GetRerencePoint(ptr->s);
            while (upper_bound < check_limit) {
                grid_map::Position pos;
                pos(0) =
                    ref_point.x() + upper_bound * cos(ptr->heading + M_PI_2);
                pos(1) =
                    ref_point.y() + upper_bound * sin(ptr->heading + M_PI_2);
                if (frame->GridMap()->IsInside(pos) &&
                    frame->GridMap()->GetObstacleDistance(pos) >
                        search_threshold) {
                    upper_bound += check_s;
                } else {
                    upper_bound -= check_s;
                    break;
                }
            }
            while (lower_bound > -check_limit) {
                grid_map::Position pos;
                pos(0) =
                    ref_point.x() + lower_bound * cos(ptr->heading + M_PI_2);
                pos(1) =
                    ref_point.y() + lower_bound * sin(ptr->heading + M_PI_2);
                if (frame->GridMap()->IsInside(pos) &&
                    frame->GridMap()->GetObstacleDistance(pos) >
                        search_threshold) {
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
    dp_layers_s_list_.resize(layers_bounds_.size());
    return true;
}

void QPSplineReferenceLineSmoother::DPCalculateCost(
    std::vector<std::vector<DP_POINT>> &samples, int layer_index,
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

bool QPSplineReferenceLineSmoother::Smooth(ReferenceLine *reference_line) {
    auto point_num = dp_layers_s_list_.size();
    if (point_num < 4) {
        ROS_WARN("ref is too short: %lu, quit POST SMOOTHING.", point_num);
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

    double s = 0;
    for (int i = 0; i < point_num; i++) {
        const double ref_s = dp_layers_s_list_[i];
        ReferencePoint ref_point = reference_line->GetRerencePoint(ref_s);
        double ref_dir = ref_point.theta();
        x_list.push_back(ref_point.x() + qpsolution(i) * cos(ref_dir + M_PI_2));
        y_list.push_back(ref_point.y() + qpsolution(i) * sin(ref_dir + M_PI_2));
        if (i > 0) {
            s += sqrt(pow(x_list[i] - x_list[i - 1], 2) +
                      pow(y_list[i] - y_list[i - 1], 2));
        }
        s_list.push_back(s);
    }
    tk::spline new_x_s, new_y_s;
    new_x_s.set_points(s_list, x_list);
    new_y_s.set_points(s_list, y_list);
    reference_line->SetSpline(new_x_s, new_y_s, s_list.back());

    return true;
}

void QPSplineReferenceLineSmoother::SetPostHessianMatrix(
    Eigen::SparseMatrix<double> *matrix_h) {
    size_t size = dp_layers_s_list_.size();
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

void QPSplineReferenceLineSmoother::SetPostConstraintMatrix(
    Eigen::SparseMatrix<double> *matrix_constraints,
    Eigen::VectorXd *lower_bound, Eigen::VectorXd *upper_bound) const {
    size_t size = dp_layers_s_list_.size();
    const int x_index = 0;
    const int dx_index = x_index + size;
    const int ddx_index = dx_index + size;
    const int cons_x_index = 0;
    const int cons_dx_x_index = cons_x_index + size;
    const int cons_ddx_dx_index = cons_dx_x_index + size - 1;
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
            -(dp_layers_s_list_[i + 1] - dp_layers_s_list_[i]);
    }
    // ddl range
    for (int i = 0; i < size - 1; ++i) {
        cons(cons_ddx_dx_index + i, dx_index + i) = -1;
        cons(cons_ddx_dx_index + i, dx_index + i + 1) = 1;
        cons(cons_ddx_dx_index + i, ddx_index + i) =
            -(dp_layers_s_list_[i + 1] - dp_layers_s_list_[i]);
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

} // namespace planning
} // namespace mujianhua
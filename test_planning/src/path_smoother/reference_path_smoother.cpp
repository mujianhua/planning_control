#include "path_smoother/reference_path_smoother.h"
#include "path_smoother/tension_smoother.h"

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

    Smooth(reference_path);
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

} // namespace planning
} // namespace mujianhua

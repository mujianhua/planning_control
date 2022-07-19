#include "path_smoother/reference_path_smoother.h"
#include "path_smoother/tension_smoother.h"

namespace mujianhua {
namespace planning {

ReferencePathSmoother::ReferencePathSmoother(
    const std::vector<TrajectoryPoint> &initial_points,
    const TrajectoryPoint &start_point)
    : initial_points_(initial_points), start_point_(start_point) {}

std::unique_ptr<ReferencePathSmoother>
ReferencePathSmoother::Creat(std::string &type,
                             const std::vector<TrajectoryPoint> &initial_points,
                             const TrajectoryPoint &start_point) {
    return std::unique_ptr<ReferencePathSmoother>{
        new TensionSmoother(initial_points, start_point)};
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

} // namespace planning
} // namespace mujianhua

#include "path_smoother/reference_path_smoother.h"
#include <cmath>
#include <cstddef>
#include <vector>
#include <glog/logging.h>
#include "config/planning_flags.h"
#include "grid_map_core/TypeDefs.hpp"
#include "math/math_util.h"
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

    GraphSearchDp(reference_path);

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
    int start_lateral_index = static_cast<int>(
        (FLAGS_search_lateral_range + vehicle_local.path_point.y) /
        FLAGS_search_lateral_spacing);
    std::vector<std::vector<DPPoint>> samples;
    samples.reserve(layers_s_list_.size());

    for (int i = 0; i < layers_s_list_.size(); ++i) {
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
                    layer_point_set[j].rough_lower_bound;
            }
        }
        for (size_t j = layer_point_set.size() - 1; j >= 0; j--) {
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
    for (const auto &layer : samples) {
        for (const auto &point : layer) {
        }
    }
    return true;
}

} // namespace planning
} // namespace mujianhua

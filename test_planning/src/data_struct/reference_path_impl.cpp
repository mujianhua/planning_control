#include "data_struct/reference_path_impl.h"
#include <config/planning_flags.h>
#include <math/math_util.h>

namespace mujianhua {
namespace planning {

ReferencePathImpl::ReferencePathImpl()
    : x_s_(new tk::spline), y_s_(new tk::spline), original_x_s_(new tk::spline),
      original_y_s_(new tk::spline) {}

ReferencePathImpl::~ReferencePathImpl() {
    delete x_s_;
    delete y_s_;
    delete original_x_s_;
    delete original_y_s_;
}

void ReferencePathImpl::clear() { reference_points_.clear(); }

void ReferencePathImpl::SetSpline(const tk::spline &x_s, const tk::spline &y_s,
                                  double max_s) {
    *x_s_ = x_s;
    *y_s_ = y_s;
    max_s_ = max_s;
}

const tk::spline &ReferencePathImpl::GetXS() const { return *x_s_; }
const tk::spline &ReferencePathImpl::GetYS() const { return *y_s_; }

double ReferencePathImpl::GetLength() const { return max_s_; }

void ReferencePathImpl::SetLength(double s) { max_s_ = s; }

bool ReferencePathImpl::BuildReferenceFromSpline(double delta_s_smaller,
                                                 double delta_s_larger) {
    static const double large_k = 0.2;
    static const double small_k = 0.08;
    reference_points_.clear();
    double tmp_s = 0.0;
    while (tmp_s <= max_s_) {
        TrajectoryPoint point;
        point.path_point.x = (*x_s_)(tmp_s);
        point.path_point.y = (*y_s_)(tmp_s);
        point.path_point.theta = math::GetHeading(*x_s_, *y_s_, tmp_s);
        point.path_point.kappa = math::GetCurvature(*x_s_, *y_s_, tmp_s);
        reference_points_.emplace_back(point);
        // TODO: ???
        if (FLAGS_enable_dynamic_segmentation) {
            double k_share =
                fabs(point.path_point.kappa) > large_k
                    ? 1.0
                    : fabs(point.path_point.kappa) < small_k
                          ? 0
                          : (fabs(point.path_point.kappa) - small_k) /
                                (large_k - small_k);
            tmp_s +=
                delta_s_larger - k_share * (delta_s_larger - delta_s_smaller);
        } else {
            tmp_s += delta_s_larger;
        }
    }
    return true;
}

void ReferencePathImpl::UpdateBoundsImproved(const Map &map) {
    if (reference_points_.empty()) {
        LOG(WARNING) << "Empty reference, updateBounds fail!";
        return;
    }
    bounds_.clear();
    VehicleBound vehicle_bound;
    for (const auto &point : reference_points_) {
        TrajectoryPoint front_center, rear_center;
        front_center.path_point.x = point.path_point.x + FLAGS_front_length *
                                    cos(point.path_point.theta);
        front_center.path_point.y = point.path_point.y + FLAGS_front_length *
                                    sin(point.path_point.theta);
        front_center.path_point.theta = point.path_point.theta;
        rear_center.path_point.x = point.path_point.x + FLAGS_rear_length *
                                   cos(point.path_point.theta);
        rear_center.path_point.y = point.path_point.y + FLAGS_rear_length *
                                   sin(point.path_point.theta);
        rear_center.path_point.theta = point.path_point.theta;

        auto front_center_directional_projection =
            math::GetProjectPointByNewton(
                *x_s_, *y_s_, front_center.path_point.x,
                front_center.path_point.y,
                point.path_point.s + FLAGS_front_length);
        auto rear_center_directional_projection = math::GetProjectPointByNewton(
            *x_s_, *y_s_, rear_center.path_point.x, rear_center.path_point.y,
            point.path_point.s + FLAGS_rear_length);
        front_center_directional_projection.path_point.theta =
            rear_center_directional_projection.path_point.theta =
                point.path_point.theta;
        auto front_bound = GetClearanceWithDirectionStrict(
            front_center_directional_projection, map);
        auto front_offset =
            math::Global2Local(front_center,
                               front_center_directional_projection)
                .path_point.y;
        front_bound[0] += front_offset;
        front_bound[1] += front_offset;

        auto rear_bound = GetClearanceWithDirectionStrict(
            rear_center_directional_projection, map);
        auto rear_offset =
            math::Global2Local(rear_center, rear_center_directional_projection)
                .path_point.y;
        rear_bound[0] += rear_offset;
        rear_bound[1] += rear_offset;
        vehicle_bound.front.set(front_bound, front_center);
        vehicle_bound.rear.set(rear_bound, rear_center);
        if (math::isEqual(front_bound[0], front_bound[1]) ||
            math::isEqual(rear_bound[0], rear_bound[1])) {
            LOG(INFO) << "Path is blocked at s: " << point.path_point.s;
            block_bound_.reset(new VehicleBound(vehicle_bound));
            break;
        }
        bounds_.emplace_back(vehicle_bound);
    }
}
std::vector<double>
ReferencePathImpl::GetClearanceWithDirectionStrict(const TrajectoryPoint &point,
                                                   const Map &map) {
    static const double search_radius = 0.5;
    double left_bound = 0.0;
    double right_bound = 0.0;
    double delta_s = 0.3;
    double left_angle = math::ConstrainAngle(point.path_point.theta + M_PI_2);
    double right_angle = math::ConstrainAngle(point.path_point.theta - M_PI_2);
    auto n = static_cast<size_t>(6.0 / delta_s);
    grid_map::Position original_position(point.path_point.x,
                                         point.path_point.y);
    double original_clearance = map.GetObstacleDistance(original_position);
    if (original_clearance > search_radius) {
        double right_s = 0.0;
        for (size_t j = 0; j < n; ++j) {
            right_s += delta_s;
            double x = point.path_point.x + right_s * cos(right_angle);
            double y = point.path_point.y + right_s * sin(right_angle);
            grid_map::Position new_position(x, y);
            double clearance = map.GetObstacleDistance(new_position);
            if (clearance < search_radius)
                break;
        }
        double left_s = 0.0;
        for (size_t j = 0; j < n; ++j) {
            left_s += delta_s;
            double x = point.path_point.x + left_s * cos(left_angle);
            double y = point.path_point.y + left_s * sin(left_angle);
            grid_map::Position new_position(x, y);
            double clearance = map.GetObstacleDistance(new_position);
            if (clearance < search_radius)
                break;
        }
        right_bound = -(right_s - delta_s);
        left_bound = left_s - delta_s;
    } else {
        LOG(INFO) << "ref state is close to obstacles! xy: "
                  << point.path_point.x << ", " << point.path_point.y;
        return {0.0, 0.0};
    }
    // Search backward more precisely.
    double smaller_ds = 0.05;
    for (int i = 0; i < static_cast<int>(delta_s / smaller_ds); i++) {
        left_bound += smaller_ds;
        grid_map::Position new_position(
            point.path_point.x + left_bound * cos(left_angle),
            point.path_point.y + left_bound * sin(left_angle));
        if (map.GetObstacleDistance(new_position)) {
            left_bound -= smaller_ds;
            break;
        }
    }
    for (int i = 0; i < static_cast<int>(delta_s / smaller_ds); i++) {
        right_bound += smaller_ds;
        grid_map::Position new_position(
            point.path_point.x + right_bound * cos(right_angle),
            point.path_point.y + right_bound * sin(right_angle));
        if (map.GetObstacleDistance(new_position)) {
            right_bound += smaller_ds;
            break;
        }
    }
    auto diff_radius = FLAGS_car_width * 0.5 - search_radius;
    left_bound -= diff_radius;
    right_bound += diff_radius;
    if (left_bound < right_bound)
        return {0.0, 0.0};
    // Hard safety margin.
    const auto space = left_angle - right_bound;
    static const auto min_space = 0.2;
    const auto max_safety_margin = std::max(0.0, (space - min_space) / 2.0);
    const auto safety_margin = std::min(FLAGS_safety_margin, max_safety_margin);
    left_bound -= safety_margin;
    right_bound += safety_margin;
    return {left_bound, right_bound};
}

} // namespace planning
} // namespace mujianhua

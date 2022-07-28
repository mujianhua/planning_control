#include "reference_line/reference_line.h"
#include <cfloat>
#include <vector>
#include "common/data_struct.h"
#include "common/math/math_util.h"
#include "common/spline.h"
#include "common/vehicle_state2.h"
#include "reference_line/reference_point.h"

namespace mujianhua {
namespace planning {

ReferenceLine::ReferenceLine()
    : x_s_(new tk::spline), y_s_(new tk::spline), block_bound_(nullptr) {}

ReferenceLine::ReferenceLine(const tk::spline &x_s, const tk::spline &y_s,
                             double max_s)
    : max_s_(max_s) {
    *x_s_ = x_s;
    *y_s_ = y_s;
}

const std::vector<PathPoint> &ReferenceLine::reference_points() const {
    return reference_points_;
}

bool ReferenceLine::BuildReferenceLineBySpline(double delta_s_smaller,
                                               double delta_s_larger) {
    static const double large_k = 0.2;
    static const double small_k = 0.08;
    reference_points_.clear();
    double tmp_s = 0.0;
    while (tmp_s <= max_s_) {
        PathPoint p((*x_s_)(tmp_s), (*y_s_)(tmp_s), GetTheta(tmp_s),
                    GetCurvature(tmp_s), tmp_s);
        reference_points_.emplace_back(p);
        // TODO: ???
        if (FLAGS_enable_dynamic_segmentation) {
            double k_share =
                fabs(p.kappa) > large_k
                    ? 1.0
                    : fabs(p.kappa) < small_k
                          ? 0
                          : (fabs(p.kappa) - small_k) / (large_k - small_k);
            tmp_s +=
                delta_s_larger - k_share * (delta_s_larger - delta_s_smaller);
        } else {
            tmp_s += delta_s_larger;
        }
    }
    return true;
}

bool ReferenceLine::BuildReferenceFromPathPoints(
    const std::vector<PathPoint> &points) {
    reference_points_.clear();
    std::vector<double> x_set, y_set, s_set;
    for (const auto &point : points) {
        reference_points_.emplace_back(point);
        x_set.emplace_back(point.x);
        y_set.emplace_back(point.y);
        s_set.emplace_back(point.s);
    }
    tk::spline x_s, y_s;
    x_s.set_points(s_set, x_set);
    y_s.set_points(s_set, y_set);
    SetSpline(x_s, y_s, s_set.back());

    return true;
}

bool ReferenceLine::UpdateBounds(Frame *frame) {
    if (reference_points_.empty()) {
        ROS_ERROR("Empty reference, updateBounds fail!");
        return false;
    }
    bounds_.clear();
    VehicleBound2 vehicle_bound;
    for (const auto &point : reference_points_) {
        VehicleState2 front_center, rear_center;

        // TrajectoryPoint front_center, rear_center;
        front_center.x() = point.x + FLAGS_front_length * cos(point.theta);
        front_center.y() = point.y + FLAGS_front_length * sin(point.theta);
        front_center.heading() = point.theta;
        rear_center.x() = point.x + FLAGS_rear_length * cos(point.theta);
        rear_center.y() = point.y + FLAGS_rear_length * sin(point.theta);
        rear_center.heading() = point.theta;

        auto front_center_directional_projection = FindProjectPointByNewton(
            front_center.x(), front_center.y(), point.s + FLAGS_front_length);
        auto rear_center_directional_projection = FindProjectPointByNewton(
            rear_center.x(), rear_center.y(), point.s + FLAGS_rear_length);

        auto front_bound = GetClearanceWithDirectionStrict(
            front_center_directional_projection, frame);
        auto front_offset =
            math::Global2Local(front_center_directional_projection,
                               front_center)
                .y();
        front_bound[0] += front_offset;
        front_bound[1] += front_offset;

        auto rear_bound = GetClearanceWithDirectionStrict(
            rear_center_directional_projection, frame);
        auto rear_offset =
            math::Global2Local(rear_center_directional_projection, rear_center)
                .y();
        rear_bound[0] += rear_offset;
        rear_bound[1] += rear_offset;
        vehicle_bound.front.set(front_bound, front_center);
        vehicle_bound.rear.set(rear_bound, rear_center);
        if (math::isEqual(front_bound[0], front_bound[1]) ||
            math::isEqual(rear_bound[0], rear_bound[1])) {
            ROS_INFO("Path is blocked at s: %f", point.s);
            block_bound_.reset(new VehicleBound2(vehicle_bound));
            break;
        }
        bounds_.emplace_back(vehicle_bound);
    }
    return true;
}

void ReferenceLine::SetSpline(const tk::spline &x_s, const tk::spline &y_s,
                              double max_s) {
    *x_s_ = x_s;
    *y_s_ = y_s;
    max_s_ = max_s;
}

const double ReferenceLine::GetLength() const { return max_s_; }

void ReferenceLine::SetLength(double s) { max_s_ = s; }

ReferencePoint ReferenceLine::FindProjectPoint(const double &xxx,
                                               const double &yyy,
                                               double start_s) {
    static const double grid = 1.0;

    // over max s of path
    if (start_s > max_s_) {
        return {(*x_s_)(start_s), (*y_s_)(start_s), start_s, GetTheta(start_s)};
    }
    double min_distance_s = start_s;
    auto min_distance = DBL_MAX;
    for (double ss = start_s; ss <= max_s_; ss += grid) {
        double tmp_x = (*x_s_)(ss);
        double tmp_y = (*y_s_)(ss);
        double distance = math::Distance(tmp_x, tmp_y, xxx, yyy);
        if (distance < min_distance) {
            min_distance_s = ss;
            min_distance = distance;
        }
    }
    double max_s_distance =
        math::Distance((*x_s_)(max_s_), (*y_s_)(max_s_), xxx, yyy);
    // end point
    if (max_s_distance < min_distance) {
        return {(*x_s_)(max_s_), (*y_s_)(max_s_), max_s_distance,
                GetTheta(max_s_distance)};
    }
    return FindProjectPointByNewton(xxx, yyy, min_distance_s);
}

ReferencePoint ReferenceLine::FindProjectPointByNewton(const double &xxx,
                                                       const double &yyy,
                                                       double cur_s) {
    double pre_s = cur_s;
    for (int i = 0; i < 20; i++) {
        double x = (*x_s_)(cur_s);
        double y = (*y_s_)(cur_s);
        double dx = (*x_s_).deriv(1, cur_s);
        double dy = (*y_s_).deriv(1, cur_s);
        double ddx = (*x_s_).deriv(2, cur_s);
        double ddy = (*y_s_).deriv(2, cur_s);
        double j = (x - xxx) * dx + (y - yyy) * dy;
        double h = dx * dx + (x - xxx) * ddx + dy * dy + (y - yyy) * ddy;
        cur_s -= j / h;
        if (fabs(cur_s - pre_s) < 1e-5) {
            break;
        }
        pre_s = cur_s;
    }
    return {(*x_s_)(cur_s), (*y_s_)(cur_s), cur_s, GetTheta(cur_s)};
}

const ReferencePoint ReferenceLine::GetRerencePoint(const double &s) const {
    return {(*x_s_)(s), (*y_s_)(s), s, GetTheta(s), GetCurvature(s)};
}

double ReferenceLine::GetTheta(double s) const {
    double x_d1 = (*x_s_).deriv(1, s);
    double y_d1 = (*y_s_).deriv(1, s);
    return atan2(y_d1, x_d1);
}

double ReferenceLine::GetCurvature(double s) const {
    double dx = (*x_s_).deriv(1, s);
    double dy = (*y_s_).deriv(1, s);
    double ddx = (*x_s_).deriv(2, s);
    double ddy = (*y_s_).deriv(2, s);
    return (dx * ddy - ddx * dy) / pow(dx * dx + dy * dy, 1.5);
}

std::vector<double>
ReferenceLine::GetClearanceWithDirectionStrict(const ReferencePoint &point,
                                               const Frame *frame) {
    static const double search_radius = 0.5;
    double left_bound = 0.0;
    double right_bound = 0.0;
    double delta_s = 0.3;
    double left_angle = math::ConstrainAngle(point.theta() + M_PI_2);
    double right_angle = math::ConstrainAngle(point.theta() - M_PI_2);
    auto n = static_cast<size_t>(6.0 / delta_s);
    grid_map::Position original_position(point.x(), point.y());
    double original_clearance =
        frame->GridMap()->GetObstacleDistance(original_position);
    if (original_clearance > search_radius) {
        double right_s = 0.0;
        for (size_t j = 0; j < n; ++j) {
            right_s += delta_s;
            double x = point.x() + right_s * cos(right_angle);
            double y = point.y() + right_s * sin(right_angle);
            grid_map::Position new_position(x, y);
            double clearance =
                frame->GridMap()->GetObstacleDistance(new_position);
            if (clearance < search_radius)
                break;
        }
        double left_s = 0.0;
        for (size_t j = 0; j < n; ++j) {
            left_s += delta_s;
            double x = point.x() + left_s * cos(left_angle);
            double y = point.y() + left_s * sin(left_angle);
            grid_map::Position new_position(x, y);
            double clearance =
                frame->GridMap()->GetObstacleDistance(new_position);
            if (clearance < search_radius)
                break;
        }
        right_bound = -(right_s - delta_s);
        left_bound = left_s - delta_s;
    } else {
        ROS_INFO("ref state is close to obstacles! xy: %f, %f", point.x(),
                 point.y());
        return {0.0, 0.0};
    }
    // Search backward more precisely.
    double smaller_ds = 0.05;
    for (int i = 0; i < static_cast<int>(delta_s / smaller_ds); i++) {
        left_bound += smaller_ds;
        grid_map::Position new_position(
            point.x() + left_bound * cos(left_angle),
            point.y() + left_bound * sin(left_angle));
        if (frame->GridMap()->GetObstacleDistance(new_position) <
            search_radius) {
            left_bound -= smaller_ds;
            break;
        }
    }
    for (int i = 0; i < static_cast<int>(delta_s / smaller_ds); i++) {
        right_bound += smaller_ds;
        grid_map::Position new_position(
            point.x() + right_bound * cos(right_angle),
            point.y() + right_bound * sin(right_angle));
        if (frame->GridMap()->GetObstacleDistance(new_position) <
            search_radius) {
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

void ReferenceLine::Clear() {
    reference_points_.clear();
    bounds_.clear();
}

const size_t ReferenceLine::size() const { return reference_points_.size(); }

const std::vector<VehicleBound2> &ReferenceLine::GetBounds() const {
    return bounds_;
}

std::shared_ptr<VehicleBound2> ReferenceLine::IsBlocked() const {
    return block_bound_;
}

} // namespace planning
} // namespace mujianhua
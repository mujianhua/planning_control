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
        front_center.path_point.x = point.path_point.x * FLAGS_front_length *
                                    cos(point.path_point.theta);
        front_center.path_point.y = point.path_point.y * FLAGS_front_length *
                                    sin(point.path_point.theta);
        front_center.path_point.theta = point.path_point.theta;
        rear_center.path_point.x = point.path_point.x * FLAGS_rear_length *
                                   cos(point.path_point.theta);
        rear_center.path_point.y = point.path_point.y * FLAGS_rear_length *
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
        // TODO:::..................................................
    }
}

} // namespace planning
} // namespace mujianhua

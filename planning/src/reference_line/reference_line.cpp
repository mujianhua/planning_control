#include "reference_line/reference_line.h"
#include <algorithm>
#include "common/math/math_utils.h"

namespace mujianhua {
namespace planning {

ReferenceLine::ReferenceLine(const std::vector<TrajectoryPoint> &points)
    : reference_points_(points) {}

TrajectoryPoint LinearInterpolateTrajectory(const TrajectoryPoint &p0,
                                            const TrajectoryPoint &p1,
                                            double s) {
    double s0 = p0.s;
    double s1 = p1.s;
    if (std::abs(s1 - s0) < common::math::kMathEpsilon) {
        return p0;
    }

    TrajectoryPoint pt;
    double weight = (s - s0) / (s1 - s0);
    pt.s = s;
    pt.x = (1 - weight) * p0.x + weight * p1.x;
    pt.y = (1 - weight) * p0.y + weight * p1.y;
    pt.theta = common::math::slerp(p0.theta, p0.s, p1.theta, p1.s, s);
    pt.kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
    pt.velocity = (1 - weight) * p0.velocity + weight * p1.velocity;
    pt.left_bound = (1 - weight) * p0.left_bound + weight * p1.left_bound;
    pt.right_bound = (1 - weight) * p0.right_bound + weight * p1.right_bound;

    return pt;
}

TrajectoryPoint ReferenceLine::EvaluateStation(double station) const {
    auto iter = QueryLowerBoundStationPoint(station);

    if (iter == reference_points_.begin()) {
        iter = std::next(iter);
    }

    auto prev = std::prev(iter, 1);

    return LinearInterpolateTrajectory(*prev, *iter, station);
}

ReferenceLine::ReferencePoints::const_iterator
ReferenceLine::QueryLowerBoundStationPoint(double station) const {
    if (station >= reference_points_.back().s) {
        return reference_points_.end() - 1;
    } else if (station < reference_points_.front().s) {
        return reference_points_.begin();
    }
    return std::lower_bound(
        reference_points_.begin(), reference_points_.end(), station,
        [](const TrajectoryPoint &t, double station) { return t.s < station; });
}

common::math::Vec2d ReferenceLine::GetCartesian(double station,
                                                double lateral) const {
    auto ref = EvaluateStation(station);
    return {ref.x - lateral * sin(ref.theta), ref.y + lateral * cos(ref.theta)};
}

} // namespace planning
} // namespace mujianhua
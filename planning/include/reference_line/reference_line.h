#pragma once

#include <vector>
#include "common/math/vec2d.h"

namespace mujianhua {
namespace planning {

struct TrajectoryPoint {
    double s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;

    double velocity = 0.0;

    double left_bound = 0.0;
    double right_bound = 0.0;
};

class ReferenceLine {
  public:
    using ReferencePoints = std::vector<TrajectoryPoint>;

    ReferenceLine() = default;

    ReferenceLine(const ReferencePoints &points);

    void SetRoadBarrier();

    const std::vector<math::Vec2d> *GetRoadBarrier() const {
        return &road_barrier_;
    }

    math::Vec2d GetProjection(const math::Vec2d &xy) const;

    TrajectoryPoint EvaluateStation(double station) const;

    ReferencePoints::const_iterator
    QueryNearestPoint(const math::Vec2d &point,
                      double *out_distance = nullptr) const;

    ReferencePoints::const_iterator
    QueryLowerBoundStationPoint(double station) const;

    math::Vec2d GetCartesian(double station, double lateral) const;

    inline const ReferencePoints &reference_points() const {
        return reference_points_;
    }

  private:
    ReferencePoints reference_points_;

    std::vector<math::Vec2d> road_barrier_;
};

} // namespace planning
} // namespace mujianhua
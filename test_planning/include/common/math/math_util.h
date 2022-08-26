#pragma once

#include <cmath>

#include "../../../../planning/src/common/data_struct.h"
#include "common/spline.h"
#include "common/vehicle_state.h"
#include "common_me/TrajectoryPoint.h"
#include "config/planning_flags.h"
#include "gflags/gflags.h"
#include "reference_line/reference_point.h"

namespace mujianhua {
namespace planning {
namespace math {

using common_me::TrajectoryPoint;

/**
 * @brief normalize angle to [-pi, pi]
 */
double NormalizeAngle(const double angle);

template <typename T> T ConstrainAngle(T angle) {
    if (angle > M_PI) {
        angle -= 2 * M_PI;
        return ConstrainAngle(angle);
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
        return ConstrainAngle(angle);
    } else {
        return angle;
    }
}

bool isEqual(double a, double b);

double Distance(const TrajectoryPoint &p1, const TrajectoryPoint &p2);

double Distance(const ReferencePoint &point, const VehicleState &state);

double Distance(const double &x1, const double &y1, const double &x2,
                const double &y2);

double Distance(const PathPoint &p1, const PathPoint &p2);

TrajectoryPoint Global2Local(const TrajectoryPoint &reference,
                             const TrajectoryPoint &target);

ReferencePoint Global2Local(const ReferencePoint &reference_point,
                            const VehicleState &vehicle_state);

PathPoint Local2Global(const PathPoint &reference_point,
                       const PathPoint &vehicle_point);

} // namespace math
} // namespace planning
} // namespace mujianhua
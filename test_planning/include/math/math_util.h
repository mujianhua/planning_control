#pragma once

#include <cmath>
#include "common/spline.h"
#include "common/vehicle_state2.h"
#include "common_me/TrajectoryPoint.h"
#include "config/planning_flags.h"
#include "data_struct/data_struct.h"
#include "gflags/gflags.h"
#include "reference_line/reference_point.h"

namespace mujianhua {
namespace planning {
namespace math {

using common_me::TrajectoryPoint;

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

double Distance(const ReferencePoint &point, const VehicleState2 &state);

double Distance(const double &x1, const double &y1, const double &x2,
                const double &y2);

double GetHeading(const tk::spline &x_s, const tk::spline &y_s, double s);

double GetCurvature(const tk::spline &x_s, const tk::spline &y_s, double s);

TrajectoryPoint GetProjectPoint(const tk::spline &x_s, const tk::spline &y_s,
                                double target_x, double target_y, double max_s,
                                double start_s = 0.0);

TrajectoryPoint GetProjectPointByNewton(const tk::spline &x_s,
                                        const tk::spline &y_s, double target_x,
                                        double target_y, double hint_s);

TrajectoryPoint Global2Local(const TrajectoryPoint &reference,
                             const TrajectoryPoint &target);

ReferencePoint Global2Local(const ReferencePoint &reference_point,
                            const VehicleState2 &vehicle_state);

PathPoint Local2Global(const PathPoint &reference_point,
                       const PathPoint &vehicle_point);

} // namespace math
} // namespace planning
} // namespace mujianhua
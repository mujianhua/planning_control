#pragma once

#include <cmath>
#include "common_me/TrajectoryPoint.h"
#include "tools/spline.h"

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

double Distance(const TrajectoryPoint &p1, const TrajectoryPoint &p2);

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

} // namespace math
} // namespace planning
} // namespace mujianhua
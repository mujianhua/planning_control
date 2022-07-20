#pragma once

#include <cmath>
#include "common_me/TrajectoryPoint.h"
#include "tools/spline.h"

namespace mujianhua {
namespace planning {
namespace math {

using common_me::TrajectoryPoint;

double Distance(const TrajectoryPoint &p1, const TrajectoryPoint &p2);

double Distance(const double &x1, const double &y1, const double &x2,
                const double &y2);

TrajectoryPoint GetProjectPoint(const tk::spline &xs, const tk::spline &ys,
                                double target_x, double target_y,
                                double start_s, double max_s);

TrajectoryPoint GetProjectPointByNewton(const tk::spline &xs, const tk::spline &ys,
                                        double target_x, double target_y,
                                        double hint_s, double max_s);

} // namespace math
} // namespace planning
} // namespace mujianhua
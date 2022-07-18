#pragma once

#include <cmath>
#include "common_me/TrajectoryPoint.h"

namespace mujianhua {
namespace planning {
namespace math {

using common_me::TrajectoryPoint;

double Distance(const TrajectoryPoint &p1, const TrajectoryPoint &p2);

} // namespace math
} // namespace planning
} // namespace mujianhua
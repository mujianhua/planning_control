/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "linear_interpolation.h"

#include "math_utils.h"

namespace planning {
namespace math {

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s) {
  double s0 = p0.s;
  double s1 = p1.s;

  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x + weight * p1.x;
  double y = (1 - weight) * p0.y + weight * p1.y;
  double theta = slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  double kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
  double dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;

  return PathPoint{s, x, y, theta, kappa, dkappa};
}

Pose InterpolateUsingLinearApproximation(const std::pair<double, Pose> &p0,
                                         const std::pair<double, Pose> &p1,
                                         const double t) {
  double t0 = p0.first;
  double t1 = p1.first;

  double weight = (t - t0) / (t1 - t0);
  double x = (1 - weight) * p0.second.x();
  double y = (1 - weight) * p0.second.y();
  double theta = (1 - weight) * p0.second.theta();
  return {x, y, theta};
}

}  // namespace math
}  // namespace planning

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

/**
 * @file
 * @brief Linear interpolation functions.
 */

#pragma once

#include <cmath>

#include "common/data_struct.h"
#include "glog/logging.h"
#include "math/pose.h"

namespace planning {
namespace math {

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1, double s);

// (t, pose)
Pose InterpolateUsingLinearApproximation(const std::pair<double, Pose> &p0,
                                         const std::pair<double, Pose> &p1,
                                         double t);

}  // namespace math
}  // namespace planning

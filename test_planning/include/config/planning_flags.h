/**
 * @file
 * @brief
 */

#pragma once

#include <gflags/gflags.h>

namespace mujianhua {
namespace planning {

DECLARE_double(car_width);

DECLARE_double(search_longitudial_spacing);

DECLARE_double(search_lateral_spacing);

DECLARE_double(search_lateral_range);

DECLARE_bool(compute_time_output);

DECLARE_string(smoothing_method);

} // namespace planning
} // namespace mujianhua
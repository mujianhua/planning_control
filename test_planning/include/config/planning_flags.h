/**
 * @file
 * @brief
 */

#pragma once

#include <gflags/gflags.h>

namespace mujianhua {
namespace planning {

DECLARE_double(car_width);

DECLARE_bool(compute_time_output);

DECLARE_string(smoothing_method);

DECLARE_double(search_longitudial_spacing);

} // namespace planning
} // namespace mujianhua
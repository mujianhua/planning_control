/**
 * @file
 * @brief
 */

#pragma once

#include <gflags/gflags.h>

namespace mujianhua {
namespace planning {

DECLARE_double(car_width);

DECLARE_double(front_length);

DECLARE_double(rear_length);

DECLARE_double(search_longitudial_spacing);

DECLARE_double(search_lateral_spacing);

DECLARE_double(search_lateral_range);

DECLARE_double(safety_margin);

DECLARE_double(epsilon);

DECLARE_double(output_spacing);

DECLARE_bool(compute_time_output);

DECLARE_bool(enable_dynamic_segmentation);

DECLARE_string(smoothing_method);

} // namespace planning
} // namespace mujianhua
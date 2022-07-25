/**
 * @file
 * @brief
 */

#pragma once

#include <cmath>
#include <gflags/gflags.h>

namespace mujianhua {
namespace planning {

DECLARE_double(car_width);

DECLARE_double(front_length);

DECLARE_double(rear_length);

DECLARE_double(wheel_base);

DECLARE_double(search_longitudial_spacing);

DECLARE_double(search_lateral_spacing);

DECLARE_double(search_lateral_range);

DECLARE_double(safety_margin);

DECLARE_double(expected_safety_margin);

DECLARE_double(epsilon);

DECLARE_double(output_spacing);

DECLARE_double(max_steering_angle);

DECLARE_bool(compute_time_output);

DECLARE_bool(enable_dynamic_segmentation);

DECLARE_bool(constraint_end_heading);

DECLARE_string(smoothing_method);

} // namespace planning
} // namespace mujianhua
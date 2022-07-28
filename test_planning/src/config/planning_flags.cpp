/**
 * @file
 */

#include "config/planning_flags.h"
#include <gflags/gflags.h>

namespace mujianhua {
namespace planning {

DEFINE_double(car_length, 4.9, "");

DEFINE_double(car_width, 2.0, "");

DEFINE_double(search_longitudial_spacing, 1.5,
              "longitudinal spacing when searching");

DEFINE_double(rear_length, -1.0, "rear axle to rear edge");

DEFINE_double(front_length, 3.9, "rear axle to front edge");

DEFINE_double(wheel_base, 2.5, "wheel base");

DEFINE_double(search_lateral_spacing, 0.6, "lateral spacing when searching");

DEFINE_double(search_lateral_range, 10.0, "lateral range when searching");

DEFINE_double(safety_margin, 0.3, "mandatory safety margin");

DEFINE_double(epsilon, 1e-6, "use this when comparing double");

DEFINE_double(output_spacing, 0.3, "output interval");

DEFINE_double(max_steering_angle, 35.0 * M_PI / 180.0, "");

DEFINE_double(expected_safety_margin, 0.6,
              "soft constraint on the distance to obstacles");

DEFINE_bool(compute_time_output, true, "Output compute detail on screen.");

DEFINE_bool(enable_dynamic_segmentation, true,
            "dense segmentation when the curvature is large.");

DEFINE_bool(constraint_end_heading, true, "add constraints on end heading");

DEFINE_string(smoothing_method, "TENSION2", "Path smoothing method.");

} // namespace planning
} // namespace mujianhua
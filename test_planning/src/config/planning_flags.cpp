/**
 * @file
 */

#include "config/planning_flags.h"

namespace mujianhua {
namespace planning {

DEFINE_double(car_width, 2.0, "");

DEFINE_bool(compute_time_output, true, "Output compute detail on screen.");

DEFINE_string(smoothing_method, "TENSION2", "Path smoothing method.");

} // namespace planning
} // namespace mujianhua
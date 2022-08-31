#include "planning/planning_config.h"

namespace planning {

DEFINE_string(planner, "Cartesian", "planner type.");

DEFINE_bool(enable_reference_line_provider_thread, true,
            "reference line provider thread");

DEFINE_bool(enable_smooth_reference_line, true, "smooth reference line");

DEFINE_double(trajectory_time_length, 8.0, "Trajectory time length");

}  // namespace planning

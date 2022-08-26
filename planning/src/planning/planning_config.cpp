#include "planning_config.h"

#include <gflags/gflags.h>

namespace planning {

DEFINE_string(planner, "Cartesian", "planner type");

DEFINE_bool(enable_reference_line_provider_thread, true,
            "reference line provider thread");

DEFINE_bool(enable_smooth_reference_line, true, "smooth reference line");

}  // namespace planning

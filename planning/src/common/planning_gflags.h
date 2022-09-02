#pragma once

#include "gflags/gflags.h"

namespace planning {

DECLARE_string(planner);

DECLARE_bool(enable_reference_line_provider_thread);

DECLARE_bool(enable_smooth_reference_line);

DECLARE_double(trajectory_time_length);
DECLARE_double(trajectory_time_resolution);
DECLARE_double(bound_buffer);
DECLARE_double(numerical_epsilon);
DECLARE_double(nudge_buffer);
DECLARE_double(max_s_lateral_optimization);
DECLARE_double(default_delta_s_lateral_optimization);

}  // namespace planning

#include "common/planning_gflags.h"

namespace planning {

DEFINE_double(trajectory_time_resolution, 0.1,
              "Trajectory time resolution in planning");
DEFINE_string(planner, "Cartesian", "planner type.");

DEFINE_bool(enable_reference_line_provider_thread, true,
            "reference line provider thread");

DEFINE_bool(enable_smooth_reference_line, true, "smooth reference line");

DEFINE_double(trajectory_time_length, 8.0, "Trajectory time length");

DEFINE_bool(lateral_optimization, true,
            "whether using optimization for lateral trajectory generation");

DEFINE_double(bound_buffer, 0.1, "buffer to boundary for lateral optimization");
DEFINE_double(numerical_epsilon, 1e-6, "Epsilon in lattice planner.");
DEFINE_double(nudge_buffer, 0.3, "buffer to nudge for lateral optimization");
DEFINE_double(max_s_lateral_optimization, 60.0,
              "The maximal s for lateral optimization.");
DEFINE_double(default_delta_s_lateral_optimization, 1.0,
              "The default delta s for lateral optimization.");
DEFINE_double(weight_lateral_offset, 1.0,
              "weight for lateral offset "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_derivative, 500.0,
              "weight for lateral derivative "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_second_order_derivative, 1000.0,
              "weight for lateral second order derivative "
              "in lateral trajectory optimization");
DEFINE_double(weight_lateral_third_order_derivative, 1000.0,
              "weight for lateral third order derivative "
              "in lateral trajectory optimization");
DEFINE_double(
    weight_lateral_obstacle_distance, 0.0,
    "weight for lateral obstacle distance in lateral trajectory optimization");

}  // namespace planning

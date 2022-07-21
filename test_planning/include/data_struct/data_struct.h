/**
 * @file
 * @brief
 */

#include <cfloat>

namespace mujianhua {
namespace planning {

struct DPPoint {
    double x, y, heading, s, l, dir, dis_to_obs;
    double cost = DBL_MAX;
    double rough_upper_bound, rough_lower_bound;
    int layer_index, lateral_index;
    const DPPoint *parent = nullptr;
    bool is_feasible = true;
};

} // namespace planning
} // namespace mujianhua
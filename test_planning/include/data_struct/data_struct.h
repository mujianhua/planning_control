/**
 * @file
 * @brief
 */

#pragma once

#include <cfloat>
#include <vector>
#include "common_me/TrajectoryPoint.h"

namespace mujianhua {
namespace planning {

using common_me::TrajectoryPoint;

struct DPPoint {
    double x{}, y{}, heading{}, s{}, l{}, dir{}, dis_to_obs{};
    double cost = DBL_MAX;
    double rough_upper_bound{}, rough_lower_bound{};
    int layer_index{}, lateral_index{};
    const DPPoint *parent = nullptr;
    bool is_feasible = true;
};

/**
 * @brief
 */
struct VehicleBound {
    VehicleBound() = default;
    struct SingleBound {
        SingleBound() = default;
        SingleBound &operator=(const std::vector<double> &bounds) {
            ub = bounds[0];
            lb = bounds[1];
        }
        void set(const std::vector<double> &bounds,
                 const TrajectoryPoint &center) {
            ub = bounds[0];
            lb = bounds[1];
            x = center.path_point.x;
            y = center.path_point.y;
            heading = center.path_point.theta;
        }

        double ub{}; // left
        double lb{}; // right
        double x{}, y{}, heading{};
    } front, rear;
};

} // namespace planning
} // namespace mujianhua
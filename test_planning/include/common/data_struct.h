/**
 * @file
 * @brief
 */

#pragma once

#include <cfloat>
#include <vector>
#include "common/vehicle_state.h"
#include "common_me/TrajectoryPoint.h"

namespace mujianhua {
namespace planning {

using common_me::TrajectoryPoint;

struct PathPoint {
    PathPoint() = default;
    PathPoint(double x, double y, double theta = 0.0, double kappa = 0.0,
              double s = 0.0)
        : x(x), y(y), theta(theta), kappa(kappa), s(s) {}

    double x{}, y{}, theta{}, s{}, kappa{};
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
                 const VehicleState &center) {
            ub = bounds[0];
            lb = bounds[1];
            x = center.x();
            y = center.y();
            heading = center.heading();
        }

        double ub{}; // left
        double lb{}; // right
        double x{}, y{}, heading{};
    } front, rear;
};

} // namespace planning
} // namespace mujianhua

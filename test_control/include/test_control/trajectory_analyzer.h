#pragma once

#include "DiscretizedTrajectory.h"
#include "test_control/simple_mpc_debug.h"

namespace mujianhua {
namespace control {

class TrajectoryAnalyzer {
  public:
    TrajectoryAnalyzer() = default;

    TrajectoryAnalyzer(const DiscretizedTrajectory *trajectory);

    /**
     * @brief Deconstructor
     */
    ~TrajectoryAnalyzer() { delete trajectory_; }

    PathPoint
    QueryNearestPointByPosition(const double x, const double y,
                                test_control::simple_mpc_debug *debug) const;

    double CalculateLaterlError(size_t index_min, const double x,
                                const double y) const;

  private:
    const DiscretizedTrajectory *trajectory_;
};

} // namespace control
} // namespace mujianhua
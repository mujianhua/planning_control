#pragma once

#include <vector>

namespace mujianhua {
namespace planning {

struct TrajectoryPoint {
    double s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;

    double velocity = 0.0;

    double left_bound = 0.0;
    double right_bound = 0.0;
};

class ReferenceLine {
  public:
    ReferenceLine() = default;

    ReferenceLine(const std::vector<TrajectoryPoint> &points);

  private:
    std::vector<TrajectoryPoint> reference_points_;
};

} // namespace planning
} // namespace mujianhua
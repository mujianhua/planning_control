#pragma once
#include <ros/ros.h>

#include <fstream>
#include <string>
#include <vector>

namespace mujianhua {
namespace control {

struct PathPoint {
    double x;
    double y;
    double theta;
    double kappa;
    double s;
};
class DiscretizedTrajectory {
  public:
    explicit DiscretizedTrajectory();

    std::vector<PathPoint> PathPoints;
};

} // namespace control
} // namespace mujianhua
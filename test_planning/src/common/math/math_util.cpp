#include "common/math/math_util.h"
#include <cmath>
#include "common/vehicle_state.h"
#include "common_me/TrajectoryPoint.h"
#include "reference_line/reference_point.h"

namespace mujianhua {
namespace planning {
namespace math {

bool isEqual(double a, double b) { return fabs(a - b) < FLAGS_epsilon; }

double Distance(const TrajectoryPoint &point1, const TrajectoryPoint &point2) {
    return sqrt(pow(point1.path_point.x - point2.path_point.x, 2) +
                pow(point1.path_point.y - point2.path_point.y, 2));
}

double Distance(const double &x1, const double &y1, const double &x2,
                const double &y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

double Distance(const ReferencePoint &point, const VehicleState &state) {
    return sqrt(pow(point.x() - state.x(), 2) + pow(point.y() - state.y(), 2));
}

double Distance(const PathPoint &p1, const PathPoint &p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

TrajectoryPoint Global2Local(const TrajectoryPoint &reference,
                             const TrajectoryPoint &target) {
    TrajectoryPoint point;
    double dx = target.path_point.x - reference.path_point.x;
    double dy = target.path_point.y - reference.path_point.y;
    point.path_point.x = dx * cos(reference.path_point.theta) +
                         dy * sin(reference.path_point.theta);
    point.path_point.y = dy * cos(reference.path_point.theta) -
                         dx * sin(reference.path_point.theta);
    point.path_point.theta =
        target.path_point.theta - reference.path_point.theta;
    point.path_point.kappa = target.path_point.kappa;
    return point;
}

ReferencePoint Global2Local(const ReferencePoint &reference_point,
                            const VehicleState &vehicle_state) {
    double dx = vehicle_state.x() - reference_point.x();
    double dy = vehicle_state.y() - reference_point.y();
    return {
        dx * cos(reference_point.theta()) + dy * sin(reference_point.theta()),
        dy * cos(reference_point.theta()) - dx * sin(reference_point.theta()),
        reference_point.s(), vehicle_state.heading() - reference_point.theta()};
}

PathPoint Local2Global(const PathPoint &reference_point,
                       const PathPoint &vehicle_point) {
    double x = vehicle_point.x * cos(reference_point.theta) -
               vehicle_point.y * sin(reference_point.theta) + reference_point.x;
    double y = vehicle_point.x * sin(reference_point.theta) +
               vehicle_point.y * cos(reference_point.theta) + reference_point.y;
    double theta = reference_point.theta + vehicle_point.theta;
    return {x, y, theta, vehicle_point.kappa, vehicle_point.s};
}

} // namespace math
} // namespace planning
} // namespace mujianhua

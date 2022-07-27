#include "math/math_util.h"
#include <cmath>
#include "common/vehicle_state2.h"
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

double GetHeading(const tk::spline &x_s, const tk::spline &y_s, double s) {
    double x_d1 = x_s.deriv(1, s);
    double y_d1 = y_s.deriv(1, s);
    return atan2(y_d1, x_d1);
}

double GetCurvature(const tk::spline &x_s, const tk::spline &y_s, double s) {
    double dx = x_s.deriv(1, s);
    double dy = y_s.deriv(1, s);
    double ddx = x_s.deriv(2, s);
    double ddy = y_s.deriv(2, s);
    return (dx * ddy - ddx * dy) / pow(dx * dx + dy * dy, 1.5);
}

TrajectoryPoint GetProjectPoint(const tk::spline &x_s, const tk::spline &y_s,
                                double target_x, double target_y, double max_s,
                                double start_s) {
    static const double grid = 1.0;
    TrajectoryPoint point;
    // over max s of path
    if (start_s > max_s) {
        point.path_point.x = x_s(start_s);
        point.path_point.y = y_s(start_s);
        point.path_point.theta = GetHeading(x_s, y_s, start_s);
        return point;
    }
    double min_distance_s = start_s;
    auto min_distance = DBL_MAX;
    for (double ss = start_s; ss <= max_s; ss += grid) {
        double tmp_x = x_s(ss);
        double tmp_y = y_s(ss);
        double distance = Distance(tmp_x, tmp_y, target_x, target_y);
        if (distance < min_distance) {
            min_distance_s = ss;
            min_distance = distance;
        }
    }
    double max_s_distance =
        Distance(x_s(max_s), y_s(max_s), target_x, target_y);
    // end point
    if (max_s_distance < min_distance) {
        point.path_point.x = x_s(max_s);
        point.path_point.y = y_s(max_s);
        point.path_point.theta = GetHeading(x_s, y_s, max_s_distance);
        return point;
    }
    return GetProjectPointByNewton(x_s, y_s, target_x, target_y,
                                   min_distance_s);
}

TrajectoryPoint GetProjectPointByNewton(const tk::spline &x_s,
                                        const tk::spline &y_s, double target_x,
                                        double target_y, double cur_s) {
    // TODO:牛顿梯度下降法
    double pre_s = cur_s;
    for (int i = 0; i < 20; i++) {
        double x = x_s(cur_s);
        double y = y_s(cur_s);
        double dx = x_s.deriv(1, cur_s);
        double dy = y_s.deriv(1, cur_s);
        double ddx = x_s.deriv(2, cur_s);
        double ddy = y_s.deriv(2, cur_s);
        double j = (x - target_x) * dx + (y - target_y) * dy;
        double h =
            dx * dx + (x - target_x) * ddx + dy * dy + (y - target_y) * ddy;
        cur_s -= j / h;
        if (fabs(cur_s - pre_s) < 1e-5) {
            break;
        }
        pre_s = cur_s;
    }
    TrajectoryPoint point;
    point.path_point.x = x_s(cur_s);
    point.path_point.y = y_s(cur_s);
    point.path_point.theta = GetHeading(x_s, y_s, cur_s);
    point.path_point.s = cur_s;
    return point;
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
                            const VehicleState2 &vehicle_state) {
    double dx = vehicle_state.x() - reference_point.x();
    double dy = vehicle_state.y() - reference_point.y();
    return {
        dx * cos(reference_point.theta()) + dy * sin(reference_point.theta()),
        dy * cos(reference_point.theta()) - dx * sin(reference_point.theta()),
        reference_point.s(), vehicle_state.heading() - reference_point.theta()};
}

} // namespace math
} // namespace planning
} // namespace mujianhua

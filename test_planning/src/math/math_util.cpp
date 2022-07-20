#include "math/math_util.h"

namespace mujianhua {
namespace planning {
namespace math {

double Distance(const TrajectoryPoint &point1, const TrajectoryPoint &point2) {
    return sqrt(pow(point1.path_point.x - point2.path_point.x, 2) +
                pow(point1.path_point.y - point2.path_point.y, 2));
}

double Distance(const double &x1, const double &y1, const double &x2,
                const double &y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

TrajectoryPoint GetProjectPoint(const tk::spline &xs, const tk::spline &ys,
                                double target_x, double target_y,
                                double start_s, double max_s) {
    static const double grid = 1.0;
    TrajectoryPoint point;
    // over max s of path
    if (start_s > max_s) {
        point.path_point.x = xs(start_s);
        point.path_point.y = ys(start_s);
        return point;
    }
    double min_distance_s = start_s;
    auto min_distance = DBL_MAX;
    for (double s = start_s; s <= max_s; s += grid) {
        double tmp_x = xs(s);
        double tmp_y = ys(s);
        double distance = Distance(tmp_x, tmp_y, target_x, target_y);
        if (distance < min_distance) {
            min_distance_s = s;
            min_distance = distance;
        }
    }
    double max_s_distance = Distance(xs(max_s), ys(max_s), target_x, target_y);
    // end point
    if (max_s_distance < min_distance) {
        point.path_point.x = xs(max_s);
        point.path_point.y = ys(max_s);
        return point;
    }
    return GetProjectPointByNewton(xs, ys, target_x, target_y, min_distance_s,
                                   max_s);
}

TrajectoryPoint GetProjectPointByNewton(const tk::spline &xs,
                                        const tk::spline &ys, double target_x,
                                        double target_y, double hint_s,
                                        double max_s) {}

} // namespace math
} // namespace planning
} // namespace mujianhua

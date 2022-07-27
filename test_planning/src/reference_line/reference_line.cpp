#include "reference_line/reference_line.h"
#include <cfloat>
#include "math/math_util.h"
#include "reference_line/reference_point.h"

namespace mujianhua {
namespace planning {

ReferenceLine::ReferenceLine(const tk::spline &x_s, const tk::spline &y_s,
                             double max_s)
    : x_s_(x_s), y_s_(y_s), max_s_(max_s) {}

const std::vector<ReferencePoint> &ReferenceLine::reference_points() const {
    return reference_points_;
}

void ReferenceLine::SetSpline(const tk::spline &x_s, const tk::spline &y_s,
                              double max_s) {
    x_s_ = x_s;
    y_s_ = y_s;
    max_s_ = max_s;
}

const double ReferenceLine::GetLength() const { return max_s_; }

ReferencePoint ReferenceLine::FindProjectPoint(const double &xxx,
                                               const double &yyy,
                                               double start_s) {
    static const double grid = 1.0;

    // over max s of path
    if (start_s > max_s_) {
        return {x_s_(start_s), y_s_(start_s), start_s, GetTheta(start_s)};
    }
    double min_distance_s = start_s;
    auto min_distance = DBL_MAX;
    for (double ss = start_s; ss <= max_s_; ss += grid) {
        double tmp_x = x_s_(ss);
        double tmp_y = y_s_(ss);
        double distance = math::Distance(tmp_x, tmp_y, xxx, yyy);
        if (distance < min_distance) {
            min_distance_s = ss;
            min_distance = distance;
        }
    }
    double max_s_distance =
        math::Distance(x_s_(max_s_), y_s_(max_s_), xxx, yyy);
    // end point
    if (max_s_distance < min_distance) {
        return {x_s_(max_s_), y_s_(max_s_), max_s_distance,
                GetTheta(max_s_distance)};
    }
    return FindProjectPointByNewton(xxx, yyy, min_distance_s);
}

ReferencePoint ReferenceLine::FindProjectPointByNewton(const double &xxx,
                                                       const double &yyy,
                                                       double cur_s) {
    double pre_s = cur_s;
    for (int i = 0; i < 20; i++) {
        double x = x_s_(cur_s);
        double y = y_s_(cur_s);
        double dx = x_s_.deriv(1, cur_s);
        double dy = y_s_.deriv(1, cur_s);
        double ddx = x_s_.deriv(2, cur_s);
        double ddy = y_s_.deriv(2, cur_s);
        double j = (x - xxx) * dx + (y - yyy) * dy;
        double h = dx * dx + (x - xxx) * ddx + dy * dy + (y - yyy) * ddy;
        cur_s -= j / h;
        if (fabs(cur_s - pre_s) < 1e-5) {
            break;
        }
        pre_s = cur_s;
    }
    return {x_s_(cur_s), y_s_(cur_s), cur_s, GetTheta(cur_s)};
}

const ReferencePoint ReferenceLine::GetRerencePoint(const double &s) const {
    return {x_s_(s), y_s_(s), s, GetTheta(s), GetCurvature(s)};
}

double ReferenceLine::GetTheta(double s) const {
    double x_d1 = x_s_.deriv(1, s);
    double y_d1 = y_s_.deriv(1, s);
    return atan2(y_d1, x_d1);
}

double ReferenceLine::GetCurvature(double s) const {
    double dx = x_s_.deriv(1, s);
    double dy = y_s_.deriv(1, s);
    double ddx = x_s_.deriv(2, s);
    double ddy = y_s_.deriv(2, s);
    return (dx * ddy - ddx * dy) / pow(dx * dx + dy * dy, 1.5);
}

} // namespace planning
} // namespace mujianhua
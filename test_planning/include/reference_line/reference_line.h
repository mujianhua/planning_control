#pragma once

#include <vector>
#include "common/spline.h"
#include "reference_line/reference_point.h"

namespace mujianhua {
namespace planning {

class ReferenceLine {
  public:
    ReferenceLine() = default;

    explicit ReferenceLine(const std::vector<ReferencePoint> &reference_points);

    explicit ReferenceLine(const tk::spline &x_s, const tk::spline &y_s,
                           double max_s);

    const std::vector<ReferencePoint> &reference_points() const;

    void SetSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);

    const double GetLength() const;

    ReferencePoint FindProjectPoint(const double &x, const double &y,
                                    double start_s = 0.0);

    ReferencePoint FindProjectPointByNewton(const double &x, const double &y,
                                            double cur_s);

    const ReferencePoint GetRerencePoint(const double &s) const;

  private:
    double GetCurvature(double s) const;
    double GetTheta(double s) const;

    std::vector<ReferencePoint> reference_points_;
    tk::spline x_s_, y_s_;
    double max_s_;
};

} // namespace planning
} // namespace mujianhua

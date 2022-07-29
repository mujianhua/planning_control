#pragma once

#include <cstddef>
#include <vector>
#include "common/data_struct.h"
#include "common/frame.h"
#include "common/spline.h"
#include "reference_line/reference_point.h"

namespace mujianhua {
namespace planning {

class ReferenceLine {
  public:
    ReferenceLine();

    explicit ReferenceLine(const std::vector<ReferencePoint> &reference_points);

    explicit ReferenceLine(const tk::spline &x_s, const tk::spline &y_s,
                           double max_s);

    bool BuildReferenceLineBySpline(double delta_s_smaller,
                                    double delta_s_larger);

    bool BuildReferenceFromPathPoints(const std::vector<PathPoint> &points);

    bool UpdateBounds(Frame *frame);

    const std::vector<PathPoint> &reference_points() const;

    void SetSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);

    const double GetLength() const;

    void SetLength(double s);

    ReferencePoint FindProjectPoint(const double &x, const double &y,
                                    double start_s = 0.0);

    ReferencePoint FindProjectPointByNewton(const double &x, const double &y,
                                            double cur_s);

    const ReferencePoint GetRerencePoint(const double &s) const;

    const std::vector<VehicleBound> &GetBounds() const;

    std::shared_ptr<VehicleBound> IsBlocked() const;

    void Clear();

    const size_t size() const;

  private:
    std::vector<double>
    GetClearanceWithDirectionStrict(const ReferencePoint &point,
                                    const Frame *frame);
    double GetCurvature(double s) const;
    double GetTheta(double s) const;

    std::vector<PathPoint> reference_points_;
    tk::spline *x_s_, *y_s_;
    double max_s_;

    std::vector<VehicleBound> bounds_;
    std::shared_ptr<VehicleBound> block_bound_;
};

} // namespace planning
} // namespace mujianhua

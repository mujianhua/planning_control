/**
 * @file
 * @brief
 */

#pragma once

#include <memory>
#include <gflags/gflags.h>
#include "data_struct/data_struct.h"
#include "data_struct/reference_path_impl.h"

namespace mujianhua {
namespace planning {

class ReferencePath {
  public:
    ReferencePath();

    void clear();

    void SetSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);

    const tk::spline &GetXS() const;
    const tk::spline &GetYS() const;

    double GetXS(double s) const;
    double GetYS(double s) const;

    double GetLength() const;

    size_t GetSize() const;

    const std::vector<TrajectoryPoint> &GetReferencePoints() const;

    void SetLength(double s);

    bool BuildReferenceFromSpline(double delta_s_smaller,
                                  double delta_s_larger);

    bool BuildReferenceFromStates(const std::vector<TrajectoryPoint> &states);

    void UpdateBounds(const Map &map);

    const std::vector<VehicleBound> &GetBounds() const;

    std::shared_ptr<VehicleBound> IsBlocked() const;

  private:
    std::shared_ptr<ReferencePathImpl> reference_path_impl_;
};

} // namespace planning
} // namespace mujianhua
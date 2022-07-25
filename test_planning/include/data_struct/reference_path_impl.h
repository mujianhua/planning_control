/**
 * @file
 * @brief
 */

#include <memory>
#include <vector>
#include <glog/logging.h>
#include "common_me/TrajectoryPoint.h"
#include "config/planning_flags.h"
#include "data_struct/data_struct.h"
#include "math/math_util.h"
#include "tools/map.h"
#include "tools/spline.h"

namespace mujianhua {
namespace planning {

using common_me::TrajectoryPoint;

class ReferencePathImpl {
  public:
    ReferencePathImpl();
    ~ReferencePathImpl();
    void clear();
    void SetSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
    const tk::spline &GetXS() const;
    const tk::spline &GetYS() const;

    double GetLength() const;

    size_t GetSize() const;

    const std::vector<TrajectoryPoint> &GetReferencePoints() const;

    void SetLength(double s);

    bool BuildReferenceFromSpline(double delta_s_smaller,
                                  double delta_s_larger);

    bool BuildReferenceFromStates(const std::vector<TrajectoryPoint> &states);

    void UpdateBoundsImproved(const Map &map);

    std::vector<double>
    GetClearanceWithDirectionStrict(const TrajectoryPoint &point,
                                    const Map &map);

    const std::vector<VehicleBound> &GetBounds() const;

    std::shared_ptr<VehicleBound> IsBlocked() const;

  private:
    std::vector<TrajectoryPoint> reference_points_;
    tk::spline *x_s_;
    tk::spline *y_s_;
    double max_s_{};
    tk::spline *original_x_s_;
    tk::spline *original_y_s_;
    double original_max_s_{};
    std::vector<VehicleBound> bounds_;
    std::shared_ptr<VehicleBound> block_bound_;
};

} // namespace planning
} // namespace mujianhua

/**
 * @file
 * @brief
 */

#include <vector>
#include "common_me/TrajectoryPoint.h"
#include "tools/spline.h"

namespace mujianhua {
namespace planning {

using common_me::TrajectoryPoint;

class ReferencePathImpl {
  public:
    ReferencePathImpl();
    ~ReferencePathImpl();
    void clear();

  private:
    std::vector<TrajectoryPoint> reference_points_;
    tk::spline *x_s_;
    tk::spline *y_s_;
    double max_s_{};
    tk::spline *original_x_s_;
    tk::spline *original_y_s_;
    double original_max_s_{};
};

} // namespace planning
} // namespace mujianhua

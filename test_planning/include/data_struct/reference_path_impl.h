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
    void SetSpline(const tk::spline &x_s, const tk::spline &y_s, double max_s);
    const tk::spline &GetXS() const;
    const tk::spline &GetYS() const;

    double GetLength() const;

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

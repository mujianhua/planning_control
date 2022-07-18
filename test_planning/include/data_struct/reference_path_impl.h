/**
 * @file
 * @brief
 */

#include <vector>
#include "common_me/TrajectoryPoint.h"

namespace mujianhua {
namespace planning {

class ReferencePathImpl {
  public:
    ReferencePathImpl();

  private:
    std::vector<common_me::TrajectoryPoint> reference_trajectory_points_;
};

} // namespace planning
} // namespace mujianhua

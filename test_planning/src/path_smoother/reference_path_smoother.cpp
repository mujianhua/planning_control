#include "path_smoother/reference_path_smoother.h"

#include <memory>

namespace mujianhua {
namespace planning {

ReferencePathSmoother::ReferencePathSmoother(const std::vector<TrajectoryPoint> &initial_path,
                           const TrajectoryPoint &start_point)
    : initial_path_(initial_path), start_point_(start_point) {}

std::unique_ptr<ReferencePathSmoother>
ReferencePathSmoother::Creat(std::string &type,
                    const std::vector<TrajectoryPoint> &initial_path,
                    const TrajectoryPoint &start_point) {

    return std::make_unique<ReferencePathSmoother>(initial_path, start_point);
}

} // namespace planning
} // namespace mujianhua
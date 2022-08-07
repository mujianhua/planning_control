#include "common/frame.h"
#include "common/local_view.h"

namespace mujianhua {
namespace planning {

Frame::Frame(uint32_t sequence_num, const ReferenceLine *reference_line,
             const LocalView &local_view)
    : sequence_num_(sequence_num), reference_line_(reference_line),
      local_view_(local_view) {
    obstacles_ = *local_view_.obstacles;
}

const std::vector<const Obstacle *> Frame::obstacles() const {
    return obstacles_.Items();
}

uint32_t Frame::SequenceNum() const { return sequence_num_; }

} // namespace planning
} // namespace mujianhua

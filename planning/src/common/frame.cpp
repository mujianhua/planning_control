#include "common/frame.h"

namespace mujianhua {
namespace planning {
namespace common {

Frame::Frame(uint32_t sequence_num, const ReferenceLine *reference_line)
    : sequence_num_(sequence_num), reference_line_(reference_line) {}

} // namespace common
} // namespace planning
} // namespace mujianhua
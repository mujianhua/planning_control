#pragma once

#include <cstdint>
#include "common/obstacle.h"
#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {
namespace common {

class Frame {
  public:
    Frame(uint32_t sequence_num, const ReferenceLine *reference_line);

  private:
    uint32_t sequence_num_ = 0;
    const ReferenceLine *reference_line_;
};

} // namespace common
} // namespace planning
} // namespace mujianhua
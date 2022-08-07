#pragma once

#include "common/local_view.h"
#include "common/obstacle.h"
#include "reference_line/reference_line.h"

namespace mujianhua {
namespace planning {

class Frame {
  public:
    Frame(uint32_t sequence_num, const ReferenceLine *reference_line,
          const LocalView &local_view);

    const ReferenceLine *reference_line() const { return reference_line_; }

    const std::vector<const Obstacle *> obstacles() const;

    uint32_t SequenceNum() const;

  private:
    uint32_t sequence_num_ = 0;
    const ReferenceLine *reference_line_;
    IndexedObstacles obstacles_;
    LocalView local_view_;
};

} // namespace planning
} // namespace mujianhua

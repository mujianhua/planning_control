#pragma once

#include <memory>

#include "common/data_struct.h"
#include "indexed_list.h"
#include "math/polygon2d.h"
#include "planning/obstacle.h"

namespace planning {

struct LocalView {
  std::shared_ptr<VehicleState> vehicle_state;
  std::shared_ptr<IndexedList<std::string, Obstacle>> obstacles;
};

}  // namespace planning

#pragma once

#include <memory>
#include "math/polygon2d.h"
#include "planning/data_struct.h"
#include "planning/indexed_list.h"
#include "planning/obstacle.h"

namespace planning {

struct LocalView {
  std::shared_ptr<VehicleState> vehicle_state;
  std::shared_ptr<IndexedList<std::string, Obstacle>> obstacles;
  std::shared_ptr<
      IndexedList<std::string, std::vector<std::pair<double, math::Polygon2d>>>>
      dynamic_obstacle;
  std::shared_ptr<IndexedList<std::string, math::Polygon2d>> static_obstacle;
};

}  // namespace planning


#include "st_boundary.h"

namespace planning {

STBoundary::STBoundary(
    const std::vector<std::pair<STPoint, STPoint>> &point_pairs,
    bool is_accurate_boundary) {
  std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);
  for (const auto &item : reduced_pairs) {
  }
}

}  // namespace planning

#include "math/smoothing_spline/spline_1d_constraint.h"

#include "math/smoothing_spline/spline_1d.h"

namespace planning {

Spline1dConstraint::Spline1dConstraint(const Spline1d& pss)
    : Spline1dConstraint(pss.x_knots(), pss.spline_order()) {}

Spline1dConstraint::Spline1dConstraint(const std::vector<double>& x_knots,
                                       const uint32_t spline_order)
    : x_knots_(x_knots), spline_order_(spline_order) {
  inequality_constraint_.SetIsEquality(false);
  equality_constraint_.SetIsEquality(true);
}

bool Spline1dConstraint::FilterConstraints(
    const std::vector<double>& x_coord, const std::vector<double>& lower_bound,
    const std::vector<double>& upper_bound,
    std::vector<double>* const filtered_lower_bound_x,
    std::vector<double>* const filtered_lower_bound,
    std::vector<double>* const filtered_upper_bound_x,
    std::vector<double>* const filtered_upper_bound) {
  filtered_lower_bound->clear();
  filtered_upper_bound->clear();
  filtered_lower_bound_x->clear();
  filtered_upper_bound_x->clear();

  static const double inf = std::numeric_limits<double>::infinity();

  filtered_lower_bound->reserve(lower_bound.size());
  filtered_lower_bound_x->reserve(lower_bound.size());
  filtered_upper_bound->reserve(upper_bound.size());
  filtered_upper_bound_x->reserve(upper_bound.size());

  for (uint32_t i = 0; i != lower_bound.size(); ++i) {
    if (std::isnan(lower_bound[i]) || lower_bound[i] == inf) return false;
    if (lower_bound[i] < inf && lower_bound[i] > -inf) {
      filtered_lower_bound->emplace_back(lower_bound[i]);
      filtered_lower_bound_x->emplace_back(x_coord[i]);
    }
  }
  // todo:.................................................
}

}  // namespace planning

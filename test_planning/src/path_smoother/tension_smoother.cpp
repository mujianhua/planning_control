#include "path_smoother/tension_smoother.h"

namespace mujianhua {
namespace planning {

TensionSmoother::TensionSmoother(
    const std::vector<TrajectoryPoint> &initial_points,
    const TrajectoryPoint &start_point, const Map &grid_map)
    : ReferencePathSmoother(initial_points, start_point, grid_map) {}

bool TensionSmoother::Smooth(ReferencePath *reference_path) {
    std::vector<double> x_list, y_list, s_list, heading_list, kappa_list;
    if (!SegmentRawReference(&x_list, &y_list, &s_list, &heading_list,
                             &kappa_list)) {
        ROS_ERROR("Unable segment raw reference path.");
        return false;
    }

    ROS_INFO("This is Tension Smoother smooth method.");
    return true;
}

bool TensionSmoother::IpoptSmooth(
    const std::vector<double> &x_list, const std::vector<double> &y_list,
    const std::vector<double> &s_list, const std::vector<double> &heading_list,
    const std::vector<double> &kappa_list, std::vector<double> *result_x_list,
    std::vector<double> *result_y_list, std::vector<double> *result_s_list) {

    CHECK_EQ(x_list.size(), y_list.size());
    CHECK_EQ(y_list.size(), s_list.size());
    CHECK_EQ(s_list.size(), heading_list.size());

    size_t n_vars = x_list.size();
    CPPAD_TESTVECTOR(double) vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    // bounds of variables
    CPPAD_TESTVECTOR(double) vars_lowerbound;
    CPPAD_TESTVECTOR(double) vars_upperbound;
    vars_lowerbound[0] = 0;
    vars_upperbound[0] = 0;
    vars_lowerbound[n_vars - 1] = -0.5;
    vars_upperbound[n_vars - 1] = 0.5;

    return true;
}

} // namespace planning
} // namespace mujianhua
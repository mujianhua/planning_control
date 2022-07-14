/**
 * @author mujianhua
 * @brief
 */

#include "test_control/trajectory_analyzer.h"

namespace mujianhua {
namespace control {

double PointDistanceSquare(const PathPoint &path_point, const double x,
                           const double y) {
    const double dx = path_point.x - x;
    const double dy = path_point.y - y;
    return dx * dx + dy * dy;
}

TrajectoryAnalyzer::TrajectoryAnalyzer(const ADCTrajectory *trajectory) {
    trajectory_ = trajectory;
}

// todo: 感觉这样计算不太行啊...
PathPoint TrajectoryAnalyzer::QueryNearestPointByPosition(
    const double x, const double y,
    test_control::simple_mpc_debug *debug) const {
    size_t index_min = 0;
    auto d_min = DBL_MAX;
    for (size_t i = 0; i <= trajectory_->PathPoints.size(); i++) {
        double d_temp = PointDistanceSquare(trajectory_->PathPoints[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    double lateral_error = CalculateLaterlError(index_min, x, y);
    debug->lateral_error = -1.0 * lateral_error;
    return trajectory_->PathPoints[index_min];
}

double TrajectoryAnalyzer::CalculateLaterlError(size_t index_min,
                                                const double x,
                                                const double y) const {
    double ey = 0.0;
    double x_back = trajectory_->PathPoints[index_min].x;
    double y_back = trajectory_->PathPoints[index_min].y;
    double x_next = trajectory_->PathPoints[index_min + 1].x;
    double y_next = trajectory_->PathPoints[index_min + 1].y;
    if (x_next == x_back) {
        ey = x - x_next;
    } else if (y_back == y_next) {
        ey = y_next - y;
    } else {
        double Kindex = (y_next - y_back) / (x_next - x_back);
        double bindex = y_next - Kindex * x_next;

        double K = -1.0 / Kindex;
        double b = y - K * x;
        ey = (Kindex * x + bindex - y) / std::sqrt(1.0 + Kindex * Kindex);
    }
    return ey;
}

} // namespace control
} // namespace mujianhua
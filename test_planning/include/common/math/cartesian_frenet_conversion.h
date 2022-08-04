/**
 * @brief
 */

#pragma once

#include <array>

namespace mujianhua {
namespace planning {
namespace math {

class CartesianFrenetConverter {
  public:
    CartesianFrenetConverter() = delete;

    static void cartesian_to_frenet(
        const double rs, const double rx, const double ry, const double rtheta,
        const double rkappa, const double rdkappa, const double x,
        const double y, const double v, const double a, const double theta,
        const double kappa, std::array<double, 3> *const ptr_s_condition,
        std::array<double, 3> *const ptr_d_condition);

    static void cartesian_to_frenet(const double rs, const double rx,
                                    const double ry, const double rtheta,
                                    const double x, const double y,
                                    double *ptr_s, double *ptr_d);

    static void frenet_to_cartesian(const double rs, const double rx,
                                    const double ry, const double rtheta,
                                    const double rkappa, const double rdkappa,
                                    const std::array<double, 3> &s_condition,
                                    const std::array<double, 3> &d_condition,
                                    double *const ptr_x, double *const ptr_y,
                                    double *const ptr_theta,
                                    double *const ptr_kappa,
                                    double *const ptr_v, double *const ptr_a);
};

} // namespace math
} // namespace planning
} // namespace mujianhua

#include "common/math/cartesian_frenet_conversion.h"
#include <cmath>

namespace mujianhua {
namespace planning {
namespace math {

namespace {
double NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, M_PI * 2.0);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}
} // namespace

void CartesianFrenetConverter::cartesian_to_frenet(
    const double rs, const double rx, const double ry, const double rtheta,
    const double rkappa, const double rdkappa, const double x, const double y,
    const double v, const double a, const double theta, const double kappa,
    std::array<double, 3> *const ptr_s_condition,
    std::array<double, 3> *const ptr_d_condition) {
    const double dx = x - rx;
    const double dy = y - ry;
    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);
    const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    ptr_d_condition->at(0) =
        std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);

    const double delta_theta = theta - rtheta;
    const double tan_delta_theta = std::tan(delta_theta);
    const double cos_delta_theta = std::cos(delta_theta);

    const double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);
    ptr_d_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta;

    const double kappa_r_d_prime =
        rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1);
    ptr_d_condition->at(2) =
        -kappa_r_d_prime * tan_delta_theta +
        one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
            (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

    ptr_s_condition->at(0) = rs;

    ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;

    const double delta_theta_prime =
        one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
    ptr_s_condition->at(2) =
        (a * cos_delta_theta -
         ptr_s_condition->at(1) * ptr_s_condition->at(1) *
             (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
        one_minus_kappa_r_d;
}

void CartesianFrenetConverter::cartesian_to_frenet(
    const double rs, const double rx, const double ry, const double rtheta,
    const double x, const double y, double *ptr_s, double *ptr_d) {
    const double dx = x - rx;
    const double dy = y - ry;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    *ptr_d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
    *ptr_s = rs;
}

void CartesianFrenetConverter::frenet_to_cartesian(
    const double rs, const double rx, const double ry, const double rtheta,
    const double rkappa, const double rdkappa,
    const std::array<double, 3> &s_condition,
    const std::array<double, 3> &d_condition, double *const ptr_x,
    double *const ptr_y, double *const ptr_theta, double *const ptr_kappa,
    double *const ptr_v, double *const ptr_a) {
    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    *ptr_x = rx - sin_theta_r * d_condition[0];
    *ptr_y = ry + cos_theta_r * d_condition[0];

    const double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];

    const double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
    const double delta_theta = std::atan2(d_condition[1], one_minus_kappa_r_d);
    const double cos_delta_theta = std::cos(delta_theta);

    *ptr_theta = NormalizeAngle(delta_theta + rtheta);

    const double kappa_r_d_prime =
        rdkappa * d_condition[0] + rkappa * d_condition[1];
    *ptr_kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
                   cos_delta_theta * cos_delta_theta) /
                      (one_minus_kappa_r_d) +
                  rkappa) *
                 cos_delta_theta / (one_minus_kappa_r_d);

    const double d_dot = d_condition[1] * s_condition[1];
    *ptr_v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *
                           s_condition[1] * s_condition[1] +
                       d_dot * d_dot);

    const double delta_theta_prime =
        one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa) - rkappa;
    *ptr_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
             s_condition[1] * s_condition[1] / cos_delta_theta *
                 (d_condition[1] * delta_theta_prime - kappa_r_d_prime);
}

} // namespace math
} // namespace planning
} // namespace mujianhua

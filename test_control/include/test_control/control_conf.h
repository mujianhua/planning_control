#pragma once

#include <vector>

namespace mujianhua {
namespace control {

struct ControlConf {
    double ts;
    double cf;
    double cr;
    int state_size;
    int control_size;
    int horizon;

    double max_vy;
    double max_yawrate;
    double max_rollrate;
    double max_roll;
    double max_fwa_deg;

    std::vector<int> mpc_matrix_q;
    std::vector<int> mpc_matrix_r;
    int mpc_max_iteration;
};

struct VehiclePara {
    double mass;
    double mass_s;
    double wheelbase;
    double lf;
    double lr;
    double iz;
    double ix;
    double h;
    double k_phi;
    double d_phi;
    double steer_ratio;
};

} // namespace control
} // namespace mujianhua
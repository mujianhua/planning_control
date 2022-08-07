/**
 * @author mujianhua
 * @brief
 */

#include "test_control/DiscretizedTrajectory.h"

namespace mujianhua {
namespace control {

DiscretizedTrajectory::DiscretizedTrajectory() {
    std::string line, number;
    std::ifstream f(
        "/home/mujianhua/catkin_ws/src/test_control/data/path_data.csv",
        std::ifstream::in);
    if (f.fail()) {
        ROS_ERROR("Can't open path data!");
    }
    while (std::getline(f, line)) {
        std::istringstream is(line);
        PathPoint path_point;
        for (int i = 0; i < 5; ++i) {
            std::getline(is, number, ',');
            switch (i) {
            case 0:
                path_point.x = atof(number.c_str());
                break;
            case 1:
                path_point.y = atof(number.c_str());
                break;
            case 2:
                path_point.theta = atof(number.c_str());
                break;
            case 3:
                path_point.kappa = atof(number.c_str());
                break;
            case 4:
                path_point.s = atof(number.c_str());
                break;
            }
        }
        PathPoints.push_back(path_point);
    }
}

} // namespace control
} // namespace mujianhua

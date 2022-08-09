/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source
 *codes. Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#include "planning/discretized_trajectory.h"
#include "math/math_utils.h"

#include <algorithm>

namespace planning {

DiscretizedTrajectory::DiscretizedTrajectory(const DiscretizedTrajectory &rhs,
                                             size_t begin, size_t end) {
    if (end < 0) {
        end = rhs.data_.size();
    }
    data_.resize(end - begin);
    std::copy_n(std::next(rhs.data_.begin(), begin), data_.size(),
                data_.begin());
}

} // namespace planning

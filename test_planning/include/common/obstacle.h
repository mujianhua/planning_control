#pragma once

#include <string>

namespace mujianhua {
namespace planning {

class Obstacle {
  public:
    Obstacle() = default;

    Obstacle(std::string &id, bool is_static);

  private:
    std::string id_;
    bool is_static_;
};

} // namespace planning
} // namespace mujianhua
#pragma once

#include <cstdint>
#include <vector>
#include <ros/ros.h>

namespace mujianhua {
namespace serial {

class WitSerial {
  public:
    WitSerial() = default;

    void Update(const std::vector<uint8_t> *buffer);

    void ProcessData() const;

  private:
    const std::vector<uint8_t> *buffer_;
};

} // namespace serial
} // namespace mujianhua
/**
 * @file
 * @brief
 */

#pragma once

#include <string>

namespace mujianhua {
namespace common {

enum ErrorCode {
    OK = 0,
};

class Status {
  public:
    explicit Status(ErrorCode code = ErrorCode::OK, std::string msg = "")
        : code_(code), msg_(msg) {}

    ~Status() = default;

  private:
    ErrorCode code_;
    std::string msg_;
};

} // namespace common
} // namespace mujianhua

int main() {
    mujianhua::common::ErrorCode error_code = mujianhua::common::ErrorCode::OK;

    return 0;
}
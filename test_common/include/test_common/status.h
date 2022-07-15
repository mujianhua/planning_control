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

    CONTROL_ERROR = 1000,
    CONTROL_INIT_ERROR = 1001,
    CONTROL_COMPUTE_ERROR = 1002,

    PLANNING_ERROR = 2000
};

class Status {
  public:
    explicit Status(ErrorCode code = ErrorCode::OK, std::string msg = "")
        : code_(code), msg_(msg) {}

    ~Status() = default;

    static Status OK() { return Status(); }

    bool ok() const;

    ErrorCode code() const;

    bool operator==(const Status &rh) const;

    bool operator!=(const Status &rh) const;

    const std::string &error_message() const;

  private:
    ErrorCode code_;
    std::string msg_;
};

} // namespace common
} // namespace mujianhua

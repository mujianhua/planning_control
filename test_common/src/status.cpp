/**
 * @file
 * @brief
 */

#include "test_common/status.h"

namespace mujianhua {
namespace common {

bool Status::ok() const { return code_ == ErrorCode::OK; }

ErrorCode Status::code() const { return code_; }

bool Status::operator==(const Status &rh) const {
    return (this->code_ == rh.code_) && (this->msg_ == rh.msg_);
}

bool Status::operator!=(const Status &rh) const { return !(*this == rh); }

const std::string &Status::error_message() const { return msg_; }

} // namespace common
} // namespace mujianhua
#include "serial_demo/wit_serial.h"
#include <cstdint>
#include <vector>

namespace mujianhua {
namespace serial {

void WitSerial::Update(const std::vector<uint8_t> *buffer) { buffer_ = buffer; }

void WitSerial::ProcessData() const {
    std::vector<std::vector<uint8_t>> data;
    std::vector<uint8_t> tmp;
    tmp.clear();
    data.clear();
    for (unsigned char i : *buffer_) {
        tmp.push_back(i);
        if (tmp.size() == 8) {
            data.push_back(tmp);
            tmp.clear();
            continue;
        }
    }
}

} // namespace serial
} // namespace mujianhua
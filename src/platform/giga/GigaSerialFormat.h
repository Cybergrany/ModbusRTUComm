#if defined(ARDUINO_GIGA)
#ifndef MODBUS_RTU_GIGA_SERIAL_FORMAT_H_
#define MODBUS_RTU_GIGA_SERIAL_FORMAT_H_

#include <Arduino.h>
#include <cstdint>
#include "drivers/BufferedSerial.h"

namespace modbus_rtu {
namespace platform {
namespace giga {

struct SerialFormat {
    uint8_t dataBits;
    mbed::BufferedSerial::Parity parity;
    uint8_t stopBits;
};

// Decode the Arduino-Mbed HardwareSerial bit layout. Keep this in step with
// UART::begin(baud, config): parity occupies 0x00f, stop bits 0x0f0, and data
// bits 0xf00. Older AVR-style masks silently turn SERIAL_8N1 into 8O1 here.
constexpr SerialFormat decodeSerialConfig(uint16_t config) {
    SerialFormat format{8, mbed::BufferedSerial::None, 1};

    switch (config & SERIAL_DATA_MASK) {
        case SERIAL_DATA_7:
            format.dataBits = 7;
            break;
        case SERIAL_DATA_8:
            format.dataBits = 8;
            break;
        default:
            break;
    }

    switch (config & SERIAL_PARITY_MASK) {
        case SERIAL_PARITY_EVEN:
            format.parity = mbed::BufferedSerial::Even;
            break;
        case SERIAL_PARITY_ODD:
            format.parity = mbed::BufferedSerial::Odd;
            break;
        case SERIAL_PARITY_NONE:
        default:
            format.parity = mbed::BufferedSerial::None;
            break;
    }

    switch (config & SERIAL_STOP_BIT_MASK) {
        case SERIAL_STOP_BIT_2:
            format.stopBits = 2;
            break;
        case SERIAL_STOP_BIT_1:
        default:
            format.stopBits = 1;
            break;
    }

    return format;
}

inline void applySerialConfig(mbed::BufferedSerial& serial, uint16_t config) {
    const SerialFormat format = decodeSerialConfig(config);
    serial.set_format(format.dataBits, format.parity, format.stopBits);
}

static_assert(decodeSerialConfig(SERIAL_8N1).dataBits == 8 &&
                  decodeSerialConfig(SERIAL_8N1).parity ==
                      mbed::BufferedSerial::None &&
                  decodeSerialConfig(SERIAL_8N1).stopBits == 1,
              "SERIAL_8N1 must decode as 8N1");
static_assert(decodeSerialConfig(SERIAL_8E1).dataBits == 8 &&
                  decodeSerialConfig(SERIAL_8E1).parity ==
                      mbed::BufferedSerial::Even &&
                  decodeSerialConfig(SERIAL_8E1).stopBits == 1,
              "SERIAL_8E1 must decode as 8E1");
static_assert(decodeSerialConfig(SERIAL_8N2).dataBits == 8 &&
                  decodeSerialConfig(SERIAL_8N2).parity ==
                      mbed::BufferedSerial::None &&
                  decodeSerialConfig(SERIAL_8N2).stopBits == 2,
              "SERIAL_8N2 must decode as 8N2");

}  // namespace giga
}  // namespace platform
}  // namespace modbus_rtu

#endif  // MODBUS_RTU_GIGA_SERIAL_FORMAT_H_
#endif  // ARDUINO_GIGA

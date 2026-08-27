#pragma once

#include <cstddef>
#include <cstdint>

using std::size_t;
using std::uint8_t;
using std::uint16_t;
using std::uint32_t;

enum PinName : int {
    NC = -1
};

inline PinName digitalPinToPinName(int pin) {
    return static_cast<PinName>(pin);
}

inline unsigned long micros() {
    static unsigned long now = 0;
    return ++now;
}

inline void delayMicroseconds(unsigned int) {}

#define SERIAL_DATA_MASK 0x0F00U
#define SERIAL_DATA_7 0x0700U
#define SERIAL_DATA_8 0x0800U
#define SERIAL_PARITY_MASK 0x000FU
#define SERIAL_PARITY_NONE 0x0000U
#define SERIAL_PARITY_EVEN 0x0002U
#define SERIAL_PARITY_ODD 0x0003U
#define SERIAL_STOP_BIT_MASK 0x00F0U
#define SERIAL_STOP_BIT_1 0x0010U
#define SERIAL_STOP_BIT_2 0x0020U
#define SERIAL_8N1 (SERIAL_DATA_8 | SERIAL_PARITY_NONE | SERIAL_STOP_BIT_1)
#define SERIAL_8E1 (SERIAL_DATA_8 | SERIAL_PARITY_EVEN | SERIAL_STOP_BIT_1)
#define SERIAL_8N2 (SERIAL_DATA_8 | SERIAL_PARITY_NONE | SERIAL_STOP_BIT_2)

namespace arduino {

class Stream {
public:
    virtual ~Stream() {}
    virtual int available() = 0;
    virtual int peek() = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t) = 0;
    virtual size_t write(const uint8_t*, size_t) = 0;
    virtual void flush() = 0;
};

class HardwareSerial : public Stream {
public:
    virtual void begin(unsigned long) = 0;
    virtual void begin(unsigned long, uint16_t) = 0;
    virtual void end() = 0;
    virtual operator bool() = 0;
};

} // namespace arduino

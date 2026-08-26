#pragma once

// Minimal Arduino shim for native/unit test builds.
//
// Design goals:
// 1) Keep compile surface small and deterministic.
// 2) Provide enough Arduino API compatibility for Modbus transport logic tests.
// 3) Offer controllable time progression for timing-sensitive RTU behavior.

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>

#ifndef HIGH
#define HIGH 0x1
#endif
#ifndef LOW
#define LOW 0x0
#endif

#ifndef INPUT
#define INPUT 0x0
#endif
#ifndef OUTPUT
#define OUTPUT 0x1
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP 0x2
#endif

#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef A0
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21
#endif

using std::uint8_t;
using std::uint16_t;
using std::uint32_t;
using std::size_t;

#ifndef bitRead
#define bitRead(value, bit) (((value) >> (bit)) & 0x01U)
#endif
#ifndef bitSet
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#endif
#ifndef bitClear
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#endif
#ifndef bitWrite
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#endif

#ifndef lowByte
#define lowByte(w) ((uint8_t)((w) & 0xFFU))
#endif
#ifndef highByte
#define highByte(w) ((uint8_t)(((w) >> 8) & 0xFFU))
#endif

namespace arduino_test {
inline uint64_t& clock_us();

enum class IoOperation : uint8_t {
  PinMode = 0,
  DigitalWrite,
  DelayMilliseconds,
  DelayMicroseconds,
  StreamWrite,
  StreamDrain
};

struct IoEvent {
  IoOperation operation;
  int32_t arg0;
  int32_t arg1;
  uint64_t timestamp_us;
};

inline std::vector<IoEvent>& io_events() {
  static std::vector<IoEvent> events;
  return events;
}

inline void record_io_event(IoOperation operation,
                            int32_t arg0 = 0,
                            int32_t arg1 = 0) {
  io_events().push_back(IoEvent{operation, arg0, arg1, clock_us()});
}

inline void clear_io_events() {
  io_events().clear();
}

// Monotonic fake clock backing micros()/millis() in native tests.
inline uint64_t& clock_us() {
  static uint64_t value = 0;
  return value;
}

inline uint32_t& micros_step_us() {
  static uint32_t step = 8;
  return step;
}

inline void reset_time(uint64_t start_us = 0) {
  clock_us() = start_us;
}

inline void set_micros_step(uint32_t step_us) {
  micros_step_us() = (step_us == 0) ? 1 : step_us;
}

inline void advance_us(uint64_t delta_us) {
  clock_us() += delta_us;
}

inline uint32_t now_us() {
  return static_cast<uint32_t>(clock_us() & 0xFFFFFFFFULL);
}
} // namespace arduino_test

static inline unsigned long micros() {
  // Advance on each call so busy loops make forward progress without real sleep.
  arduino_test::advance_us(arduino_test::micros_step_us());
  return arduino_test::now_us();
}

static inline unsigned long millis() {
  return static_cast<unsigned long>(arduino_test::now_us() / 1000UL);
}

static inline void delay(unsigned long ms) {
  arduino_test::record_io_event(
      arduino_test::IoOperation::DelayMilliseconds,
      static_cast<int32_t>(ms));
  arduino_test::advance_us(static_cast<uint64_t>(ms) * 1000ULL);
}

static inline void delayMicroseconds(unsigned int us) {
  arduino_test::record_io_event(
      arduino_test::IoOperation::DelayMicroseconds,
      static_cast<int32_t>(us));
  arduino_test::advance_us(us);
}

static inline void yield() {
  arduino_test::advance_us(50);
}

static inline void noInterrupts() {}
static inline void interrupts() {}

namespace arduino {

class Stream {
 public:
  virtual ~Stream() = default;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() { return -1; }
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t* buffer, size_t size) {
    size_t written = 0;
    for (size_t i = 0; i < size; ++i) {
      written += write(buffer[i]);
    }
    return written;
  }
  virtual void flush() {}
};

class HardwareSerial : public Stream {
 public:
  ~HardwareSerial() override = default;
};

} // namespace arduino

using Stream = arduino::Stream;

class TestSerialConsole {
 public:
  // Logging output is intentionally no-op in native tests.
  void print(const char*) {}
  void print(char) {}
  void print(unsigned char, int = 10) {}
  void print(unsigned int, int = 10) {}
  void print(int, int = 10) {}
  void print(unsigned long, int = 10) {}
  void print(long, int = 10) {}
  void println() {}
  void println(const char*) {}
  void println(char) {}
  void println(unsigned char, int = 10) {}
  void println(unsigned int, int = 10) {}
  void println(int, int = 10) {}
  void println(unsigned long, int = 10) {}
  void println(long, int = 10) {}
};

static TestSerialConsole Serial __attribute__((unused));

#ifndef SERIAL_8N1
#define SERIAL_8N1 0x06
#endif
#ifndef SERIAL_8N2
#define SERIAL_8N2 0x0E
#endif
#ifndef SERIAL_8E1
#define SERIAL_8E1 0x26
#endif
#ifndef SERIAL_8E2
#define SERIAL_8E2 0x2E
#endif
#ifndef SERIAL_8O1
#define SERIAL_8O1 0x36
#endif
#ifndef SERIAL_8O2
#define SERIAL_8O2 0x3E
#endif

static inline void pinMode(uint8_t pin, uint8_t mode) {
  arduino_test::record_io_event(
      arduino_test::IoOperation::PinMode,
      static_cast<int32_t>(pin),
      static_cast<int32_t>(mode));
}
static inline void digitalWrite(uint8_t pin, uint8_t value) {
  arduino_test::record_io_event(
      arduino_test::IoOperation::DigitalWrite,
      static_cast<int32_t>(pin),
      static_cast<int32_t>(value));
}
static inline int digitalRead(uint8_t) { return LOW; }
static inline void analogWrite(uint8_t, int) {}
static inline int analogRead(uint8_t) { return 0; }

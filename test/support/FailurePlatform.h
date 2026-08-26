#ifndef MODBUS_RTU_TEST_FAILURE_PLATFORM_H_
#define MODBUS_RTU_TEST_FAILURE_PLATFORM_H_

#include "Arduino.h"
#include "ModbusRTUPlatform.h"

namespace failure_platform_test {

inline bool& drainResult() {
  static bool value = true;
  return value;
}

inline void reset() {
  drainResult() = true;
}

class Backend {
 public:
  static uint32_t microsNow() { return ::micros(); }
  static uint32_t millisNow() { return ::millis(); }
  static void sleepMilliseconds(uint32_t ms) { ::delay(ms); }
  static void waitDelayMicroseconds(uint32_t us) {
    ::delayMicroseconds(static_cast<unsigned int>(us));
  }
  static void yieldTask() { ::yield(); }

  template <typename StreamType>
  static int available(StreamType& stream) { return stream.available(); }

  template <typename StreamType>
  static int read(StreamType& stream) { return stream.read(); }

  template <typename StreamType>
  static size_t write(StreamType& stream,
                      const uint8_t* bytes,
                      size_t length) {
    return stream.write(bytes, length);
  }

  template <typename StreamType>
  static bool waitForTransmitDrain(StreamType& stream,
                                   size_t,
                                   uint32_t) {
    stream.flush();
    return drainResult();
  }

  static void configureDriverPins(int8_t dePin, int8_t rePin) {
    if (dePin >= 0) {
      ::pinMode(static_cast<uint8_t>(dePin), OUTPUT);
      ::digitalWrite(static_cast<uint8_t>(dePin), LOW);
    }
    if (rePin >= 0) {
      ::pinMode(static_cast<uint8_t>(rePin), OUTPUT);
      ::digitalWrite(static_cast<uint8_t>(rePin), LOW);
    }
  }

  static void setDriverTransmit(int8_t dePin, bool enabled) {
    if (dePin >= 0) {
      ::digitalWrite(static_cast<uint8_t>(dePin), enabled ? HIGH : LOW);
    }
  }

  template <typename StreamType>
  static bool attachReadable(StreamType&,
                             modbus_rtu::platform::ReadableCallback,
                             void*) {
    return false;
  }

  template <typename StreamType>
  static void detachReadable(StreamType&, bool& attached) {
    attached = false;
  }

  static bool startEventTask(
      void*& handle,
      const modbus_rtu::platform::EventTaskConfig&,
      modbus_rtu::platform::TaskEntry,
      void*) {
    handle = nullptr;
    return false;
  }
  static bool eventTaskRunning(void*) { return false; }
  static void waitEvent(void*) {}
  static void signalEvent(void*) {}
  static void stopEventTask(void*& handle) { handle = nullptr; }
  static modbus_rtu::platform::TaskStackSnapshot currentTaskStack() {
    return modbus_rtu::platform::TaskStackSnapshot();
  }

  static void diagnosticsLock() {}
  static void diagnosticsUnlock() {}
  static bool diagnosticsTryLock() { return false; }
  static bool diagnosticsCanWrite(size_t) { return false; }
};

typedef modbus_rtu::platform::StaticModbusRTUPlatform<Backend> Platform;

}  // namespace failure_platform_test

#endif  // MODBUS_RTU_TEST_FAILURE_PLATFORM_H_

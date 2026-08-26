#ifndef ARDUINO_MODBUS_RTU_PLATFORM_H_
#define ARDUINO_MODBUS_RTU_PLATFORM_H_

// Arduino implementation of the neutral Modbus RTU platform contract.
// Authoritative contract: src/platform/README.md
// Backend-specific notes: src/platform/arduino/README.md
//
// Generic Arduino targets retain direct Stream calls and polling. Arduino
// GIGA/mbed additionally maps readable callbacks, event tasks, stack snapshots,
// and estimated TX drain timing onto the existing production primitives.

#include "Arduino.h"
#include "ModbusRTUPlatform.h"
#include "ModbusRTUDiagnosticsPolicy.h"

#include <stddef.h>
#include <stdint.h>

#ifndef MBUS_DIAG_NONBLOCKING_LOG
#define MBUS_DIAG_NONBLOCKING_LOG 1
#endif

#if defined(ARDUINO_ARCH_MBED) || defined(__MBED__)
#include "mbed.h"
#include <chrono>
#include <new>
#endif

#if defined(ARDUINO_GIGA) && __has_include("../giga/GigaBufferedSerial.h")
// The relative include is intentional: the backend must bind to the
// implementation shipped by this package, not whichever similarly named
// header appears first on the application's include path.
#include "../giga/GigaBufferedSerial.h"
#define MBUS_ARDUINO_PLATFORM_HAS_GIGA_BUFFERED_SERIAL 1
#else
#define MBUS_ARDUINO_PLATFORM_HAS_GIGA_BUFFERED_SERIAL 0
#endif

#if MBUS_RTU_PLATFORM_TRACE && !defined(MBUS_RTU_PLATFORM_TRACE_HOOK)
#define MBUS_RTU_PLATFORM_TRACE_HOOK(record) ((void)(record))
#endif

namespace modbus_rtu {
namespace platform {

#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED && \
    (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
template <typename BackendTag>
struct ArduinoModbusDiagnosticsMutexStorage {
  static rtos::Mutex mutex;
};

template <typename BackendTag>
rtos::Mutex ArduinoModbusDiagnosticsMutexStorage<BackendTag>::mutex;
#endif

class ArduinoModbusRTUPlatformBackend {
 public:
  static uint32_t microsNow() {
    return static_cast<uint32_t>(::micros());
  }

  static uint32_t millisNow() {
    return static_cast<uint32_t>(::millis());
  }

  static void sleepMilliseconds(uint32_t ms) {
#if defined(ARDUINO_ARCH_MBED) || defined(__MBED__)
    rtos::ThisThread::sleep_for(std::chrono::milliseconds(ms));
#else
    ::delay(ms);
#endif
  }

  static void waitDelayMicroseconds(uint32_t delayUs) {
    if (delayUs >= 1000U) {
      const uint32_t ms = delayUs / 1000U;
      ::delayMicroseconds(delayUs - (ms * 1000U));
      sleepMilliseconds(ms);
    } else {
      ::delayMicroseconds(delayUs);
    }
  }

  static void yieldTask() {
#if defined(ARDUINO_ARCH_MBED) || defined(__MBED__)
    rtos::ThisThread::yield();
#else
    ::yield();
#endif
  }

  template <typename StreamType>
  static int available(StreamType& stream) {
    return stream.available();
  }

  template <typename StreamType>
  static int read(StreamType& stream) {
    return stream.read();
  }

  template <typename StreamType>
  static size_t write(StreamType& stream,
                      const uint8_t* bytes,
                      size_t length) {
    return stream.write(bytes, length);
  }

  template <typename StreamType>
  static bool waitForTransmitDrain(StreamType& stream,
                                   size_t byteCount,
                                   uint32_t charTimeUs) {
#if MBUS_ARDUINO_PLATFORM_HAS_GIGA_BUFFERED_SERIAL
    // Preserve the GIGA estimate path: BufferedSerial::sync() can wedge a
    // downstream worker, while accepted bytes still have a calculable wire
    // time. This proves an elapsed estimate, not UART/RS485 electrical idle.
    return GigaBufferedSerial::waitForTxDrainEstimate(
        stream, byteCount, charTimeUs);
#else
    (void)byteCount;
    (void)charTimeUs;
    stream.flush();
    return true;
#endif
  }

  static void configureDriverPins(int8_t dePin, int8_t rePin) {
    if (dePin >= 0) {
      ::pinMode(dePin, OUTPUT);
      ::digitalWrite(dePin, LOW);
    }
    if (rePin >= 0) {
      ::pinMode(rePin, OUTPUT);
      ::digitalWrite(rePin, LOW);
    }
  }

  static void setDriverTransmit(int8_t dePin, bool enabled) {
    if (dePin >= 0) {
      ::digitalWrite(dePin, enabled ? HIGH : LOW);
    }
  }

  template <typename StreamType>
  static bool attachReadable(StreamType& stream,
                             ReadableCallback callback,
                             void* context) {
#if MBUS_ARDUINO_PLATFORM_HAS_GIGA_BUFFERED_SERIAL
    return GigaBufferedSerial::attachReadableCallback(
        stream, callback, context);
#else
    // Returning false is intentional: ModbusRTUComm keeps polling the Stream.
    (void)stream;
    (void)callback;
    (void)context;
    return false;
#endif
  }

  template <typename StreamType>
  static void detachReadable(StreamType& stream, bool& attached) {
#if MBUS_ARDUINO_PLATFORM_HAS_GIGA_BUFFERED_SERIAL
    if (attached) {
      GigaBufferedSerial::detachReadableCallback(stream);
      attached = false;
    }
#else
    (void)stream;
    (void)attached;
#endif
  }

  static bool startEventTask(void*& handle,
                             const EventTaskConfig& config,
                             TaskEntry entry,
                             void* context) {
#if defined(ARDUINO_ARCH_MBED) || defined(__MBED__)
    if (handle || !entry) {
      return false;
    }
    EventTaskState* state = new (std::nothrow) EventTaskState(config);
    if (!state) {
      return false;
    }
    state->run = true;
    // Publish before Thread::start(): sigio may report readable input in this
    // window and signalEvent() must have a valid semaphore target.
    handle = state;
    const osStatus startStatus =
        state->thread.start(mbed::callback(entry, context));
    if (startStatus != osOK) {
      // Restore the neutral failure contract so ModbusRTUComm remains on its
      // polling ingress path; never expose a failed task as running.
      state->run = false;
      handle = 0;
      delete state;
      return false;
    }
    return true;
#else
    // Non-mbed Arduino targets intentionally have no platform-owned RX task.
    (void)handle;
    (void)config;
    (void)entry;
    (void)context;
    return false;
#endif
  }

  static bool eventTaskRunning(void* handle) {
#if defined(ARDUINO_ARCH_MBED) || defined(__MBED__)
    const EventTaskState* state = static_cast<const EventTaskState*>(handle);
    return state && state->run;
#else
    (void)handle;
    return false;
#endif
  }

  static void waitEvent(void* handle) {
#if defined(ARDUINO_ARCH_MBED) || defined(__MBED__)
    EventTaskState* state = static_cast<EventTaskState*>(handle);
    if (state) {
      state->event.acquire();
    }
#else
    (void)handle;
#endif
  }

  static void signalEvent(void* handle) {
#if defined(ARDUINO_ARCH_MBED) || defined(__MBED__)
    EventTaskState* state = static_cast<EventTaskState*>(handle);
    if (state) {
      state->event.release();
    }
#else
    (void)handle;
#endif
  }

  static void stopEventTask(void*& handle) {
#if defined(ARDUINO_ARCH_MBED) || defined(__MBED__)
    EventTaskState* state = static_cast<EventTaskState*>(handle);
    if (!state) {
      return;
    }
    state->run = false;
    state->event.release();
    state->thread.join();
    delete state;
    handle = 0;
#else
    handle = 0;
#endif
  }

  static TaskStackSnapshot currentTaskStack() {
#if defined(ARDUINO_ARCH_MBED) || defined(__MBED__)
    osThreadId_t id = rtos::ThisThread::get_id();
    if (!id) {
      return TaskStackSnapshot();
    }
    return TaskStackSnapshot(
        static_cast<uint32_t>(osThreadGetStackSize(id)),
        static_cast<uint32_t>(osThreadGetStackSpace(id)),
        true);
#else
    return TaskStackSnapshot();
#endif
  }

  static void diagnosticsLock() {
#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED && \
    (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
    diagnosticsMutex().lock();
#endif
  }

  static void diagnosticsUnlock() {
#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED && \
    (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
    diagnosticsMutex().unlock();
#endif
  }

  static bool diagnosticsTryLock() {
#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED && \
    (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
    return diagnosticsMutex().trylock();
#elif MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED
    return true;
#else
    return false;
#endif
  }

  static bool diagnosticsCanWrite(size_t bytesHint) {
#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED && \
    (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
#if MBUS_DIAG_NONBLOCKING_LOG
    if (!Serial) {
      return false;
    }
    const int available = Serial.availableForWrite();
    return available > 0 && static_cast<size_t>(available) >= bytesHint;
#else
    (void)bytesHint;
    return true;
#endif
#elif MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED
    (void)bytesHint;
    return true;
#else
    (void)bytesHint;
    return false;
#endif
  }

#if MBUS_RTU_PLATFORM_TRACE
  static void trace(const TraceRecord& record) {
    MBUS_RTU_PLATFORM_TRACE_HOOK(record);
  }
#endif

 private:
#if defined(ARDUINO_ARCH_MBED) || defined(__MBED__)
  struct EventTaskState {
    rtos::Thread thread;
    rtos::Semaphore event;
    volatile bool run;

    explicit EventTaskState(const EventTaskConfig& config)
        : thread(config.priority == TaskPriority::AboveNormal
                     ? osPriorityAboveNormal
                     : osPriorityNormal,
                 config.stack_bytes,
                 0,
                 config.name),
          event(0),
          run(false) {}
  };

#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED
  static rtos::Mutex& diagnosticsMutex() {
    return ArduinoModbusDiagnosticsMutexStorage<
        ArduinoModbusRTUPlatformBackend>::mutex;
  }
#endif
#endif
};

typedef StaticModbusRTUPlatform<ArduinoModbusRTUPlatformBackend>
    ArduinoModbusRTUPlatform;

}  // namespace platform
}  // namespace modbus_rtu

#endif  // ARDUINO_MODBUS_RTU_PLATFORM_H_

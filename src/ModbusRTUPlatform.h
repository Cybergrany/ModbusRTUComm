#ifndef MODBUS_RTU_PLATFORM_H_
#define MODBUS_RTU_PLATFORM_H_

// Platform-neutral services used by the Modbus RTU transport.
//
// This header is deliberately C++11-compatible and includes no Arduino, mbed,
// RTOS, application, or concrete Stream type. StaticModbusRTUPlatform adapts a backend
// at compile time; it does not add virtual dispatch, ownership, or allocation.
// See src/platform/README.md for the complete backend contract, examples,
// supported compile gates, and hardware-validation boundary.

#include <stddef.h>
#include <stdint.h>

#ifndef MBUS_RTU_PLATFORM_TRACE
// Platform-boundary trace is characterization-only. When disabled, callers
// compile out every trace site and the production backend owns no trace state.
#define MBUS_RTU_PLATFORM_TRACE 0
#endif

namespace modbus_rtu {
namespace platform {

typedef void (*TaskEntry)(void* context);
// A readable callback is only an edge/hint that input may now be available.
// It may run asynchronously and must remain short; the transport subsequently
// drains bytes through available()/read(). The opaque context is never owned.
typedef void (*ReadableCallback)(void* context);

// Portable intent only. A backend maps this to the nearest platform priority;
// exact scheduling priority is not guaranteed by the neutral contract.
enum class TaskPriority : uint8_t {
  Normal = 0,
  AboveNormal
};

struct EventTaskConfig {
  // All fields are requests. Backends may reject unsupported configurations by
  // returning false from startEventTask(); they must not leave a live handle.
  TaskPriority priority;
  size_t stack_bytes;
  // Non-owning name pointer. A backend that retains it requires the storage to
  // outlive the task, as the Arduino/mbed backend does.
  const char* name;

  EventTaskConfig(TaskPriority taskPriority,
                  size_t stackBytes,
                  const char* taskName)
      : priority(taskPriority), stack_bytes(stackBytes), name(taskName) {}
};

struct TaskStackSnapshot {
  // Byte counts are meaningful only when valid is true. Unsupported platforms
  // return the default invalid snapshot rather than inventing a watermark.
  uint32_t size_bytes;
  uint32_t free_bytes;
  bool valid;

  TaskStackSnapshot(uint32_t sizeBytes = 0,
                    uint32_t freeBytes = 0,
                    bool isValid = false)
      : size_bytes(sizeBytes), free_bytes(freeBytes), valid(isValid) {}
};

// Wrap-safe absolute deadline for a wrapping uint32_t microsecond clock.
//
// `after()` intentionally permits unsigned wrap. Comparisons use a signed
// difference, so the requested interval and every observation must remain
// within the signed 32-bit half-range (< 2^31 us, about 35.8 minutes). A default
// Deadline targets timestamp zero; it is not an "inactive" sentinel.
class Deadline {
 public:
  Deadline() : at_us_(0) {}
  explicit Deadline(uint32_t atUs) : at_us_(atUs) {}

  static Deadline after(uint32_t nowUs, uint32_t delayUs) {
    return Deadline(nowUs + delayUs);
  }

  bool reached(uint32_t nowUs) const {
    return static_cast<int32_t>(nowUs - at_us_) >= 0;
  }

  uint32_t remaining(uint32_t nowUs) const {
    return reached(nowUs) ? 0U : static_cast<uint32_t>(at_us_ - nowUs);
  }

  uint32_t atUs() const { return at_us_; }

 private:
  uint32_t at_us_;
};

// Characterization events at the platform/transport boundary. These are not a
// wire protocol, stable telemetry schema, or replacement for hardware traces.
enum class TraceEvent : uint8_t {
  ReadableSignalled = 0,
  EventTaskWakeRequested,
  EventTaskStarted,
  EventTaskWaiting,
  EventTaskWoken,
  EventTaskStopped,
  IngressByte,
  RxStateTransition,
  TxGateOpen,
  TxWriteStarted,
  TxWriteFinished
};

struct TraceRecord {
  // detail is event-specific and diagnostic-only. Trace-enabled builds may
  // make extra clock calls and therefore intentionally perturb timing.
  TraceEvent event;
  uint32_t timestamp_us;
  uint32_t detail;

  TraceRecord(TraceEvent traceEvent,
              uint32_t timestampUs,
              uint32_t traceDetail = 0)
      : event(traceEvent), timestamp_us(timestampUs), detail(traceDetail) {}
};

// Compile-time facade over a platform Backend.
//
// Backend methods are static and platform-specific, so production byte/timing
// paths remain eligible for full inlining and carry no vtable, function table,
// or per-call type-erasure cost. Template stream operations accept the caller's
// concrete stream type without exposing it in this neutral header.
//
// A complete backend supplies every delegated method below with the same
// signature and semantics. C++ instantiates only methods that a consumer uses,
// which permits small polling-only backends, but the supplied Arduino backend
// implements the whole contract. See README.md before adding a backend.
template <typename Backend>
struct StaticModbusRTUPlatform {
  // Wrapping clocks and cooperative waits. Delay methods must not return before
  // the requested logical delay; their resolution and scheduling jitter remain
  // platform properties.
  static uint32_t microsNow() { return Backend::microsNow(); }
  static uint32_t millisNow() { return Backend::millisNow(); }
  static void sleepMilliseconds(uint32_t ms) {
    Backend::sleepMilliseconds(ms);
  }
  static void waitDelayMicroseconds(uint32_t us) {
    Backend::waitDelayMicroseconds(us);
  }
  static void yieldTask() { Backend::yieldTask(); }

  template <typename StreamType>
  static int available(StreamType& stream) {
    return Backend::available(stream);
  }

  template <typename StreamType>
  static int read(StreamType& stream) {
    return Backend::read(stream);
  }

  template <typename StreamType>
  static size_t write(StreamType& stream,
                      const uint8_t* bytes,
                      size_t length) {
    return Backend::write(stream, bytes, length);
  }

  template <typename StreamType>
  static bool waitForTransmitDrain(StreamType& stream,
                                   size_t byteCount,
                                   uint32_t charTimeUs) {
    // true means the accepted byteCount is safe to follow with the caller's
    // post-TX delay and DE-low transition. false is a transport failure.
    return Backend::waitForTransmitDrain(stream, byteCount, charTimeUs);
  }

  // Negative pins mean "not supplied" and must be harmless to a backend.
  // configureDriverPins establishes the receive-safe initial state; transmit
  // toggling is intentionally separate from stream writes and drain waiting.
  static void configureDriverPins(int8_t dePin, int8_t rePin) {
    Backend::configureDriverPins(dePin, rePin);
  }
  static void setDriverTransmit(int8_t dePin, bool enabled) {
    Backend::setDriverTransmit(dePin, enabled);
  }

  template <typename StreamType>
  static bool attachReadable(StreamType& stream,
                             ReadableCallback callback,
                             void* context) {
    // true means callbacks may occur until detachReadable(); false requests the
    // transport's polling fallback. The backend does not own stream/context.
    return Backend::attachReadable(stream, callback, context);
  }

  template <typename StreamType>
  static void detachReadable(StreamType& stream, bool& attached) {
    Backend::detachReadable(stream, attached);
  }

  static bool startEventTask(void*& handle,
                             const EventTaskConfig& config,
                             TaskEntry entry,
                             void* context) {
    // On success, publish a non-null signalable handle before making `entry`
    // runnable, then return true. This lets a readable callback queue an event
    // during startup. On failure, return false with handle null, no runnable
    // task, and no retained resources; the caller remains in polling mode.
    return Backend::startEventTask(handle, config, entry, context);
  }
  static bool eventTaskRunning(void* handle) {
    return Backend::eventTaskRunning(handle);
  }
  static void waitEvent(void* handle) { Backend::waitEvent(handle); }
  // A signal issued before the next wait must remain observable. signalEvent()
  // may be called from the readable-callback context, so its backend operation
  // must be valid there and must not perform blocking work.
  static void signalEvent(void* handle) { Backend::signalEvent(handle); }
  // Queue one wake only when both a published handle and an earlier hint exist.
  // This helper neither owns the handle nor clears the caller's pending flag.
  static bool replayPendingEvent(void* handle, bool pending) {
    if (!handle || !pending) {
      return false;
    }
    Backend::signalEvent(handle);
    return true;
  }
  // stopEventTask() must wake/join or otherwise terminate the task, release its
  // backend resources, and null the handle before returning. Callback-versus-
  // teardown exclusion is an owner/platform lifecycle responsibility.
  static void stopEventTask(void*& handle) { Backend::stopEventTask(handle); }
  static TaskStackSnapshot currentTaskStack() {
    return Backend::currentTaskStack();
  }

  // Diagnostics are optional and are not transport serialization. Disabled
  // backends use no-op locks, false try/capacity results, and no mutex storage.
  static void diagnosticsLock() { Backend::diagnosticsLock(); }
  static void diagnosticsUnlock() { Backend::diagnosticsUnlock(); }
  static bool diagnosticsTryLock() { return Backend::diagnosticsTryLock(); }
  static bool diagnosticsCanWrite(size_t bytesHint) {
    return Backend::diagnosticsCanWrite(bytesHint);
  }

#if MBUS_RTU_PLATFORM_TRACE
  // This member and its call sites exist only in trace-enabled translation
  // units. Keep MBUS_RTU_PLATFORM_TRACE consistent and define it before include.
  static void trace(const TraceRecord& record) { Backend::trace(record); }
#endif
};

}  // namespace platform
}  // namespace modbus_rtu

#endif  // MODBUS_RTU_PLATFORM_H_

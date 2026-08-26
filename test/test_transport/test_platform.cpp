#include <unity.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#ifndef MBUS_RTU_PLATFORM_TRACE
#define MBUS_RTU_PLATFORM_TRACE 1
#endif
#include "ModbusRTUPlatform.h"

namespace {

enum class BackendCall : std::uint8_t {
  Micros = 0,
  Millis,
  Sleep,
  Delay,
  Yield,
  Available,
  Read,
  Write,
  Drain,
  ConfigurePins,
  DriverTransmit,
  Attach,
  Detach,
  TaskStart,
  TaskRunning,
  EventWait,
  EventSignal,
  TaskStop,
  Stack,
  DiagLock,
  DiagUnlock,
  DiagTryLock,
  DiagCanWrite,
  Trace
};

struct FakeStream {
  int available_count = 0;
  int next_byte = -1;
  std::size_t write_count = 0;
  bool drained = true;
};

struct FakeTask {
  bool running = false;
  std::uint32_t pending = 0;
  modbus_rtu::platform::TaskEntry entry = nullptr;
  void* context = nullptr;
};

class FakeBackend {
 public:
  typedef void (*StartWindowHook)(void* context);

  static std::vector<BackendCall>& calls() {
    static std::vector<BackendCall> value;
    return value;
  }

  static FakeTask& task() {
    static FakeTask value;
    return value;
  }

  static std::uint32_t& clockUs() {
    static std::uint32_t value = 0;
    return value;
  }

  static std::uint32_t& traceDetail() {
    static std::uint32_t value = 0;
    return value;
  }

  static StartWindowHook& startWindowHook() {
    static StartWindowHook value = nullptr;
    return value;
  }

  static void*& startWindowContext() {
    static void* value = nullptr;
    return value;
  }

  static void reset() {
    calls().clear();
    task() = FakeTask{};
    clockUs() = 0;
    traceDetail() = 0;
    startWindowHook() = nullptr;
    startWindowContext() = nullptr;
  }

  static std::uint32_t microsNow() {
    calls().push_back(BackendCall::Micros);
    return clockUs();
  }
  static std::uint32_t millisNow() {
    calls().push_back(BackendCall::Millis);
    return clockUs() / 1000U;
  }
  static void sleepMilliseconds(std::uint32_t ms) {
    calls().push_back(BackendCall::Sleep);
    clockUs() += ms * 1000U;
  }
  static void waitDelayMicroseconds(std::uint32_t us) {
    calls().push_back(BackendCall::Delay);
    clockUs() += us;
  }
  static void yieldTask() { calls().push_back(BackendCall::Yield); }

  static int available(FakeStream& stream) {
    calls().push_back(BackendCall::Available);
    return stream.available_count;
  }
  static int read(FakeStream& stream) {
    calls().push_back(BackendCall::Read);
    if (stream.available_count > 0) {
      --stream.available_count;
    }
    return stream.next_byte;
  }
  static std::size_t write(FakeStream& stream,
                           const std::uint8_t*,
                           std::size_t length) {
    calls().push_back(BackendCall::Write);
    stream.write_count += length;
    return length;
  }
  static bool waitForTransmitDrain(FakeStream& stream,
                                   std::size_t,
                                   std::uint32_t) {
    calls().push_back(BackendCall::Drain);
    return stream.drained;
  }

  static void configureDriverPins(std::int8_t, std::int8_t) {
    calls().push_back(BackendCall::ConfigurePins);
  }
  static void setDriverTransmit(std::int8_t, bool) {
    calls().push_back(BackendCall::DriverTransmit);
  }
  static bool attachReadable(FakeStream&,
                             modbus_rtu::platform::ReadableCallback,
                             void*) {
    calls().push_back(BackendCall::Attach);
    return true;
  }
  static void detachReadable(FakeStream&, bool& attached) {
    calls().push_back(BackendCall::Detach);
    attached = false;
  }

  static bool startEventTask(
      void*& handle,
      const modbus_rtu::platform::EventTaskConfig&,
      modbus_rtu::platform::TaskEntry entry,
      void* context) {
    calls().push_back(BackendCall::TaskStart);
    FakeTask& value = task();
    value.running = true;
    value.entry = entry;
    value.context = context;
    handle = &value;
    if (startWindowHook()) {
      startWindowHook()(startWindowContext());
    }
    return true;
  }
  static bool eventTaskRunning(void* handle) {
    calls().push_back(BackendCall::TaskRunning);
    return handle == &task() && task().running;
  }
  static void waitEvent(void* handle) {
    calls().push_back(BackendCall::EventWait);
    if (handle == &task() && task().pending > 0) {
      --task().pending;
      task().entry(task().context);
    }
  }
  static void signalEvent(void* handle) {
    calls().push_back(BackendCall::EventSignal);
    if (handle == &task()) {
      ++task().pending;
    }
  }
  static void stopEventTask(void*& handle) {
    calls().push_back(BackendCall::TaskStop);
    task().running = false;
    task().pending = 0;
    handle = nullptr;
  }
  static modbus_rtu::platform::TaskStackSnapshot currentTaskStack() {
    calls().push_back(BackendCall::Stack);
    return modbus_rtu::platform::TaskStackSnapshot(768U, 512U, true);
  }

  static void diagnosticsLock() { calls().push_back(BackendCall::DiagLock); }
  static void diagnosticsUnlock() { calls().push_back(BackendCall::DiagUnlock); }
  static bool diagnosticsTryLock() {
    calls().push_back(BackendCall::DiagTryLock);
    return true;
  }
  static bool diagnosticsCanWrite(std::size_t) {
    calls().push_back(BackendCall::DiagCanWrite);
    return true;
  }

  static void trace(const modbus_rtu::platform::TraceRecord& record) {
    calls().push_back(BackendCall::Trace);
    traceDetail() = record.detail;
  }
};

typedef modbus_rtu::platform::StaticModbusRTUPlatform<FakeBackend> Platform;

struct StartWindowProbe {
  void** handle_location = nullptr;
  bool saw_published_handle = false;
};

void signalDuringStart(void* context) {
  StartWindowProbe* probe = static_cast<StartWindowProbe*>(context);
  probe->saw_published_handle =
      probe->handle_location && *probe->handle_location;
  if (probe->saw_published_handle) {
    Platform::signalEvent(*probe->handle_location);
  }
}

void increment(void* context) {
  std::uint32_t* count = static_cast<std::uint32_t*>(context);
  ++(*count);
}

void test_deadline_edges_are_wrap_safe_and_do_not_expire_early() {
  const modbus_rtu::platform::Deadline normal =
      modbus_rtu::platform::Deadline::after(1000U, 750U);
  TEST_ASSERT_FALSE(normal.reached(1749U));
  TEST_ASSERT_EQUAL_UINT32(1U, normal.remaining(1749U));
  TEST_ASSERT_TRUE(normal.reached(1750U));
  TEST_ASSERT_EQUAL_UINT32(0U, normal.remaining(1750U));

  const modbus_rtu::platform::Deadline wrapped =
      modbus_rtu::platform::Deadline::after(0xFFFFFFF0U, 0x30U);
  TEST_ASSERT_EQUAL_HEX32(0x20U, wrapped.atUs());
  TEST_ASSERT_FALSE(wrapped.reached(0x1FU));
  TEST_ASSERT_EQUAL_UINT32(1U, wrapped.remaining(0x1FU));
  TEST_ASSERT_TRUE(wrapped.reached(0x20U));
}

void test_event_task_preserves_signal_then_wake_order_without_coalescing() {
  FakeBackend::reset();
  void* handle = nullptr;
  std::uint32_t callbackCount = 0;
  const modbus_rtu::platform::EventTaskConfig config(
      modbus_rtu::platform::TaskPriority::AboveNormal, 768U, "rx");

  TEST_ASSERT_TRUE(Platform::startEventTask(handle, config, increment, &callbackCount));
  Platform::signalEvent(handle);
  Platform::signalEvent(handle);
  Platform::waitEvent(handle);
  Platform::waitEvent(handle);
  TEST_ASSERT_EQUAL_UINT32(2U, callbackCount);
  TEST_ASSERT_TRUE(Platform::eventTaskRunning(handle));
  Platform::stopEventTask(handle);
  TEST_ASSERT_NULL(handle);

  const std::vector<BackendCall>& calls = FakeBackend::calls();
  TEST_ASSERT_EQUAL_UINT32(7U, calls.size());
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::TaskStart),
                          static_cast<std::uint8_t>(calls[0]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::EventSignal),
                          static_cast<std::uint8_t>(calls[1]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::EventSignal),
                          static_cast<std::uint8_t>(calls[2]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::EventWait),
                          static_cast<std::uint8_t>(calls[3]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::EventWait),
                          static_cast<std::uint8_t>(calls[4]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::TaskRunning),
                          static_cast<std::uint8_t>(calls[5]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::TaskStop),
                          static_cast<std::uint8_t>(calls[6]));
}

void test_event_raised_during_start_is_queued_after_handle_publication() {
  FakeBackend::reset();
  void* handle = nullptr;
  std::uint32_t callbackCount = 0;
  StartWindowProbe probe;
  probe.handle_location = &handle;
  FakeBackend::startWindowHook() = signalDuringStart;
  FakeBackend::startWindowContext() = &probe;
  const modbus_rtu::platform::EventTaskConfig config(
      modbus_rtu::platform::TaskPriority::AboveNormal, 768U, "rx");

  TEST_ASSERT_TRUE(Platform::startEventTask(handle, config, increment, &callbackCount));
  TEST_ASSERT_TRUE(probe.saw_published_handle);
  TEST_ASSERT_EQUAL_UINT32(1U, FakeBackend::task().pending);
  Platform::waitEvent(handle);
  TEST_ASSERT_EQUAL_UINT32(1U, callbackCount);

  const std::vector<BackendCall>& calls = FakeBackend::calls();
  TEST_ASSERT_EQUAL_UINT32(3U, calls.size());
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::TaskStart),
                          static_cast<std::uint8_t>(calls[0]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::EventSignal),
                          static_cast<std::uint8_t>(calls[1]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::EventWait),
                          static_cast<std::uint8_t>(calls[2]));

  Platform::stopEventTask(handle);
}

void test_pending_event_before_handle_publication_is_replayed_after_start() {
  FakeBackend::reset();
  void* handle = nullptr;
  std::uint32_t callbackCount = 0;
  const modbus_rtu::platform::EventTaskConfig config(
      modbus_rtu::platform::TaskPriority::AboveNormal, 768U, "rx");

  // This models attachReadable() reporting an edge before startEventTask()
  // has published a handle. The first wake is necessarily unsignalable.
  TEST_ASSERT_FALSE(Platform::replayPendingEvent(handle, true));
  TEST_ASSERT_TRUE(
      Platform::startEventTask(handle, config, increment, &callbackCount));
  TEST_ASSERT_TRUE(Platform::replayPendingEvent(handle, true));
  TEST_ASSERT_EQUAL_UINT32(1U, FakeBackend::task().pending);
  Platform::waitEvent(handle);
  TEST_ASSERT_EQUAL_UINT32(1U, callbackCount);

  const std::vector<BackendCall>& calls = FakeBackend::calls();
  TEST_ASSERT_EQUAL_UINT32(3U, calls.size());
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::TaskStart),
                          static_cast<std::uint8_t>(calls[0]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::EventSignal),
                          static_cast<std::uint8_t>(calls[1]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::EventWait),
                          static_cast<std::uint8_t>(calls[2]));

  Platform::stopEventTask(handle);
}

void test_static_transport_delegation_preserves_call_order_and_results() {
  FakeBackend::reset();
  FakeStream stream;
  stream.available_count = 1;
  stream.next_byte = 0x5A;
  const std::uint8_t bytes[] = {0x11, 0x22, 0x33};

  TEST_ASSERT_EQUAL_INT(1, Platform::available(stream));
  TEST_ASSERT_EQUAL_HEX8(0x5A, Platform::read(stream));
  TEST_ASSERT_EQUAL_UINT32(3U, Platform::write(stream, bytes, sizeof(bytes)));
  TEST_ASSERT_TRUE(Platform::waitForTransmitDrain(stream, sizeof(bytes), 40U));

  const std::vector<BackendCall>& calls = FakeBackend::calls();
  TEST_ASSERT_EQUAL_UINT32(4U, calls.size());
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::Available),
                          static_cast<std::uint8_t>(calls[0]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::Read),
                          static_cast<std::uint8_t>(calls[1]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::Write),
                          static_cast<std::uint8_t>(calls[2]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::Drain),
                          static_cast<std::uint8_t>(calls[3]));
}

#if MBUS_RTU_PLATFORM_TRACE
void test_trace_forwarding_is_ordered_and_payload_exact_when_enabled() {
  FakeBackend::reset();
  Platform::trace(modbus_rtu::platform::TraceRecord(
      modbus_rtu::platform::TraceEvent::ReadableSignalled, 100U, 0x1234U));
  Platform::trace(modbus_rtu::platform::TraceRecord(
      modbus_rtu::platform::TraceEvent::EventTaskWakeRequested, 101U, 0x5678U));

  TEST_ASSERT_EQUAL_UINT32(2U, FakeBackend::calls().size());
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::Trace),
                          static_cast<std::uint8_t>(FakeBackend::calls()[0]));
  TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(BackendCall::Trace),
                          static_cast<std::uint8_t>(FakeBackend::calls()[1]));
  TEST_ASSERT_EQUAL_HEX32(0x5678U, FakeBackend::traceDetail());
}
#endif

}  // namespace

void run_modbus_rtu_platform_tests() {
  RUN_TEST(test_deadline_edges_are_wrap_safe_and_do_not_expire_early);
  RUN_TEST(test_event_task_preserves_signal_then_wake_order_without_coalescing);
  RUN_TEST(test_event_raised_during_start_is_queued_after_handle_publication);
  RUN_TEST(test_pending_event_before_handle_publication_is_replayed_after_start);
  RUN_TEST(test_static_transport_delegation_preserves_call_order_and_results);
#if MBUS_RTU_PLATFORM_TRACE
  RUN_TEST(test_trace_forwarding_is_ordered_and_payload_exact_when_enabled);
#endif
}

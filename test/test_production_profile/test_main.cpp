#include <unity.h>

#include <deque>
#include <vector>

#include "Arduino.h"
#include <ModbusADU.h>
#include <ModbusRTUComm.h>

#ifdef MBUS_RTU_TEST_BUNDLED_ADU_FIXTURE
#error "Production profile must use the exact pinned ModbusADU package"
#endif

#if MBUS_RTU_PLATFORM_TRACE != 0
#error "Production profile must compile platform trace out"
#endif

#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED != 0
#error "Production profile must compile direct serial diagnostics out"
#endif

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
#error "Production profile must compile detailed metrics out"
#endif

#if !defined(MBUS_RTU_COMM_HAS_ONE_SHOT_GAPS) || \
    MBUS_RTU_COMM_HAS_ONE_SHOT_GAPS != 1
#error "Production profile requires one-shot scheduling gaps"
#endif

#if !defined(MBUS_RTU_COMM_HAS_NO_RESPONSE_GATE) || \
    MBUS_RTU_COMM_HAS_NO_RESPONSE_GATE != 1
#error "Production profile requires no-response next-TX gating"
#endif

static_assert(MBUS_RTU_RX_RING_SIZE == 256,
              "generic production polling profile uses a 256-entry RX ring");
static_assert(MBUS_RTU_ENABLE_RX_THREAD == 0,
              "generic production profile keeps RX task support disabled");
static_assert(MBUS_RTU_TIMING_MODE == MBUS_RTU_TIMING_SPEC_FIXED_GT19200,
              "production profile uses fixed high-baud T1.5/T3.5 timing");
static_assert(sizeof(ModbusRTUComm) == 3776U,
              "production native ABI footprint changed");

namespace {

class ProductionStream : public Stream {
 public:
  struct TimedByte {
    uint8_t value;
    uint32_t ready_us;

    TimedByte(uint8_t byteValue, uint32_t readyUs)
        : value(byteValue), ready_us(readyUs) {}
  };

  int available() override {
    int count = 0;
    const uint32_t now = arduino_test::now_us();
    for (const TimedByte& entry : rx) {
      if (entry.ready_us > now) {
        break;
      }
      ++count;
    }
    return count;
  }

  int read() override {
    if (rx.empty() || rx.front().ready_us > arduino_test::now_us()) {
      return -1;
    }
    const int value = rx.front().value;
    rx.pop_front();
    return value;
  }

  int peek() override {
    return (rx.empty() || rx.front().ready_us > arduino_test::now_us())
               ? -1
               : rx.front().value;
  }

  size_t write(uint8_t value) override { return write(&value, 1U); }

  size_t write(const uint8_t* bytes, size_t length) override {
    arduino_test::record_io_event(
        arduino_test::IoOperation::StreamWrite,
        static_cast<int32_t>(length),
        static_cast<int32_t>(length));
    write_times.push_back(arduino_test::clock_us());
    tx.insert(tx.end(), bytes, bytes + length);
    return length;
  }

  void flush() override {
    arduino_test::record_io_event(arduino_test::IoOperation::StreamDrain);
  }

  void push(const uint8_t* bytes,
            size_t length,
            uint32_t firstUs,
            uint32_t interUs) {
    uint32_t readyUs = firstUs;
    for (size_t i = 0; i < length; ++i) {
      rx.push_back(TimedByte(bytes[i], readyUs));
      readyUs += interUs;
    }
  }

  std::deque<TimedByte> rx;
  std::vector<uint8_t> tx;
  std::vector<uint64_t> write_times;
};

void prepareFc03(ModbusADU& request, uint8_t unit) {
  request.setUnitId(unit);
  request.setFunctionCode(0x03);
  request.setDataRegister(0, 0x006B);
  request.setDataRegister(2, 0x0003);
  request.setDataLen(4);
}

void prepareFc69(ModbusADU& request) {
  request.setUnitId(0);
  request.setFunctionCode(0x45);
  request.data[0] = 20;
  request.data[1] = 0x10;
  request.data[2] = 0;
  request.data[3] = 0;
  request.data[4] = 0;
  request.data[5] = 1;
  request.data[6] = 2;
  request.data[7] = 0x12;
  request.data[8] = 0x34;
  request.setDataLen(9);
}

std::vector<arduino_test::IoOperation> lifecycleOperations() {
  std::vector<arduino_test::IoOperation> out;
  for (const arduino_test::IoEvent& event : arduino_test::io_events()) {
    if (event.operation == arduino_test::IoOperation::DigitalWrite ||
        event.operation == arduino_test::IoOperation::StreamWrite ||
        event.operation == arduino_test::IoOperation::StreamDrain ||
        event.operation == arduino_test::IoOperation::DelayMicroseconds) {
      out.push_back(event.operation);
    }
  }
  return out;
}

void test_production_tu_exact_frames_order_timing_and_no_response_gate() {
  arduino_test::reset_time();
  arduino_test::set_micros_step(8U);
  arduino_test::clear_io_events();

  ProductionStream stream;
  const int8_t dePin = 4;
  const int8_t rePin = 5;
  ModbusRTUComm comm(stream, dePin, rePin);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(5U);

  // The production native ABI footprint is frozen alongside the explicit ring
  // profile. This detects accidental metrics/trace state or per-object backend
  // storage being introduced into a release build.
  TEST_ASSERT_EQUAL_UINT32(3776U, sizeof(ModbusRTUComm));

  arduino_test::clear_io_events();
  ModbusADU noResponse;
  prepareFc69(noResponse);
  comm.setPostTxGapUsOnce(5000U);
  TEST_ASSERT_TRUE(comm.writeAdu(noResponse));

  const uint8_t expectedFc69[] = {
      0x00, 0x45, 0x14, 0x10, 0x00, 0x00, 0x00,
      0x01, 0x02, 0x12, 0x34, 0x47, 0x67};
  TEST_ASSERT_EQUAL_UINT32(sizeof(expectedFc69), stream.tx.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(
      expectedFc69, stream.tx.data(), sizeof(expectedFc69));

  ModbusADU strict;
  prepareFc03(strict, 0x11);
  TEST_ASSERT_TRUE(comm.writeAdu(strict));
  TEST_ASSERT_EQUAL_UINT32(2U, stream.write_times.size());
  TEST_ASSERT_GREATER_OR_EQUAL_UINT64(
      1750ULL + 5000ULL,
      stream.write_times[1] - stream.write_times[0]);

  const uint8_t expectedFc03[] = {
      0x11, 0x03, 0x00, 0x6B, 0x00, 0x03, 0x76, 0x87};
  TEST_ASSERT_EQUAL_UINT32(
      sizeof(expectedFc69) + sizeof(expectedFc03), stream.tx.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(
      expectedFc03,
      stream.tx.data() + sizeof(expectedFc69),
      sizeof(expectedFc03));

  const std::vector<arduino_test::IoOperation> ops = lifecycleOperations();
  TEST_ASSERT_EQUAL_UINT32(10U, ops.size());
  for (size_t base = 0; base < ops.size(); base += 5U) {
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(arduino_test::IoOperation::DigitalWrite),
        static_cast<uint8_t>(ops[base]));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(arduino_test::IoOperation::StreamWrite),
        static_cast<uint8_t>(ops[base + 1U]));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(arduino_test::IoOperation::StreamDrain),
        static_cast<uint8_t>(ops[base + 2U]));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(arduino_test::IoOperation::DelayMicroseconds),
        static_cast<uint8_t>(ops[base + 3U]));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(arduino_test::IoOperation::DigitalWrite),
        static_cast<uint8_t>(ops[base + 4U]));
  }

  const uint8_t response[] = {
      0x11, 0x03, 0x06, 0x00, 0x01, 0x00,
      0x02, 0x00, 0x03, 0x30, 0xB4};
  stream.push(response,
              sizeof(response),
              arduino_test::now_us() + 100U,
              40U);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, comm.readAdu(strict));
  TEST_ASSERT_EQUAL_UINT32(sizeof(response), strict.getRtuLen());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(response, strict.rtu, sizeof(response));
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_production_tu_exact_frames_order_timing_and_no_response_gate);
  return UNITY_END();
}

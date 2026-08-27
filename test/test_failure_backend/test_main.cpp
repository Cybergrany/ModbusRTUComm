#include <unity.h>

#include <vector>

#include "Arduino.h"
#include "ModbusADU.h"
#define private public
#include "ModbusRTUComm.h"
#undef private

#include "ModbusADU.cpp"
#include "ModbusRTUComm.cpp"

namespace {

class FailureStream : public Stream {
 public:
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }

  size_t write(uint8_t value) override {
    return write(&value, 1U);
  }

  size_t write(const uint8_t* bytes, size_t length) override {
    arduino_test::record_io_event(
        arduino_test::IoOperation::StreamWrite,
        static_cast<int32_t>(length),
        static_cast<int32_t>(length));
    tx.insert(tx.end(), bytes, bytes + length);
    return length;
  }

  void flush() override {
    arduino_test::record_io_event(arduino_test::IoOperation::StreamDrain);
  }

  std::vector<uint8_t> tx;
};

void prepareRead(ModbusADU& request) {
  request.setUnitId(0x11);
  request.setFunctionCode(0x03);
  request.setDataRegister(0, 0x006B);
  request.setDataRegister(2, 0x0003);
  request.setDataLen(4);
}

void test_drain_failure_restores_de_and_consumes_post_gap() {
  arduino_test::reset_time();
  arduino_test::set_micros_step(8U);
  arduino_test::clear_io_events();
  failure_platform_test::reset();

  FailureStream stream;
  const int8_t dePin = 7;
  const int8_t rePin = 8;
  ModbusRTUComm comm(stream, dePin, rePin);
  comm.begin(250000, SERIAL_8N1);
  comm._postDelay = 23U;
  comm.setPostTxGapUsOnce(4000U);
  arduino_test::clear_io_events();
  failure_platform_test::drainResult() = false;

  ModbusADU request;
  prepareRead(request);
  TEST_ASSERT_FALSE(comm.writeAdu(request));

  const std::vector<arduino_test::IoEvent>& events = arduino_test::io_events();
  TEST_ASSERT_EQUAL_UINT32(5U, events.size());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::DigitalWrite),
      static_cast<uint8_t>(events[0].operation));
  TEST_ASSERT_EQUAL_INT8(dePin, events[0].arg0);
  TEST_ASSERT_EQUAL_INT(HIGH, events[0].arg1);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::StreamWrite),
      static_cast<uint8_t>(events[1].operation));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::StreamDrain),
      static_cast<uint8_t>(events[2].operation));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::DelayMicroseconds),
      static_cast<uint8_t>(events[3].operation));
  TEST_ASSERT_EQUAL_UINT32(23U, events[3].arg0);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::DigitalWrite),
      static_cast<uint8_t>(events[4].operation));
  TEST_ASSERT_EQUAL_INT8(dePin, events[4].arg0);
  TEST_ASSERT_EQUAL_INT(LOW, events[4].arg1);

  for (const arduino_test::IoEvent& event : events) {
    if (event.operation == arduino_test::IoOperation::DigitalWrite) {
      TEST_ASSERT_NOT_EQUAL(rePin, event.arg0);
    }
  }
  TEST_ASSERT_EQUAL_UINT32(0U, comm._oneShotPostGapUs);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_FRAME_ERROR,
                          comm.debugInfo().last_err);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(
      comm._frameTimeout + 4000U - 64U,
      comm._remainingTxGateUs(arduino_test::now_us()));

  const uint8_t expected[] = {
      0x11, 0x03, 0x00, 0x6B, 0x00, 0x03, 0x76, 0x87};
  TEST_ASSERT_EQUAL_UINT32(sizeof(expected), stream.tx.size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, stream.tx.data(), sizeof(expected));
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_drain_failure_restores_de_and_consumes_post_gap);
  return UNITY_END();
}

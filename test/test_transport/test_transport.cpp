#include <unity.h>

#include <deque>
#include <initializer_list>
#include <vector>

#define MBUS_DETAILED_METRICS 1
#define MBUS_RTU_DEDUPE_WINDOW_US 20000UL
#ifndef MBUS_RTU_PLATFORM_TRACE
#define MBUS_RTU_PLATFORM_TRACE 1
#endif

#if MBUS_RTU_PLATFORM_TRACE
namespace {
struct CapturedPlatformTrace {
  uint8_t event = 0;
  uint32_t timestamp_us = 0;
  uint32_t detail = 0;

  CapturedPlatformTrace(uint8_t traceEvent,
                        uint32_t timestampUs,
                        uint32_t traceDetail)
      : event(traceEvent),
        timestamp_us(timestampUs),
        detail(traceDetail) {}
};

std::vector<CapturedPlatformTrace>& capturedPlatformTrace() {
  static std::vector<CapturedPlatformTrace> records;
  return records;
}

template <typename Record>
void captureModbusPlatformTrace(const Record& record) {
  capturedPlatformTrace().push_back(CapturedPlatformTrace{
      static_cast<uint8_t>(record.event), record.timestamp_us, record.detail});
}
}  // namespace

#define MBUS_RTU_PLATFORM_TRACE_HOOK(record) captureModbusPlatformTrace(record)
#endif

#include "Arduino.h"

#include "ModbusADU.h"
#define private public
#include "ModbusRTUComm.h"
#undef private

#include "ModbusADU.cpp"
#include "ModbusRTUComm.cpp"

namespace {

// Deterministic stream shim with timestamped RX bytes. This lets the tests
// emulate UART timing edges (late arrival, T1.5 gaps, concatenated frames)
// without hardware.
class ScriptedStream : public Stream {
 public:
  enum class Operation : uint8_t {
    Available = 0,
    Read,
    Write,
    Flush
  };

  struct TimedByte {
    uint8_t value = 0;
    uint32_t ready_us = 0;

    TimedByte(uint8_t byteValue, uint32_t readyUs)
        : value(byteValue), ready_us(readyUs) {}
  };

  int available() override {
    _operations.push_back(Operation::Available);
    const uint32_t now = arduino_test::now_us();
    int count = 0;
    for (const TimedByte& entry : _rx) {
      if (entry.ready_us > now) {
        break;
      }
      ++count;
    }
    return count;
  }

  int read() override {
    _operations.push_back(Operation::Read);
    if (_rx.empty()) {
      return -1;
    }
    const uint32_t now = arduino_test::now_us();
    if (_rx.front().ready_us > now) {
      return -1;
    }
    const int out = _rx.front().value;
    _rx.pop_front();
    return out;
  }

  int peek() override {
    if (_rx.empty()) {
      return -1;
    }
    const uint32_t now = arduino_test::now_us();
    if (_rx.front().ready_us > now) {
      return -1;
    }
    return _rx.front().value;
  }

  size_t write(uint8_t value) override {
    _operations.push_back(Operation::Write);
    const size_t written = (_write_limit == 0U) ? 0U : 1U;
    arduino_test::record_io_event(
        arduino_test::IoOperation::StreamWrite,
        1,
        static_cast<int32_t>(written));
    arduino_test::advance_us(_write_cost_us);
    if (written != 0U) {
      _tx.push_back(value);
      _tx_write_times.push_back(arduino_test::now_us());
    }
    return written;
  }

  size_t write(const uint8_t* data, size_t len) override {
    _operations.push_back(Operation::Write);
    size_t written = len;
    if (!data || _write_limit < written) {
      written = data ? _write_limit : 0U;
    }
    arduino_test::record_io_event(
        arduino_test::IoOperation::StreamWrite,
        static_cast<int32_t>(len),
        static_cast<int32_t>(written));
    arduino_test::advance_us(_write_cost_us);
    if (written == 0U) {
      return 0;
    }
    _tx.insert(_tx.end(), data, data + written);
    _tx_write_times.push_back(arduino_test::now_us());
    return written;
  }

  void flush() override {
    _operations.push_back(Operation::Flush);
    arduino_test::record_io_event(arduino_test::IoOperation::StreamDrain);
    arduino_test::advance_us(_flush_cost_us);
  }

  void pushBytes(const std::vector<uint8_t>& bytes, uint32_t first_us, uint32_t inter_us) {
    uint32_t ts = first_us;
    for (uint8_t b : bytes) {
      _rx.push_back(TimedByte{b, ts});
      ts += inter_us;
    }
  }

  void clear() {
    _rx.clear();
    _tx.clear();
    _tx_write_times.clear();
    _operations.clear();
  }

  const std::vector<uint8_t>& txBytes() const { return _tx; }
  const std::vector<Operation>& operations() const { return _operations; }
  void setWriteCostUs(uint32_t value) { _write_cost_us = value; }
  void setFlushCostUs(uint32_t value) { _flush_cost_us = value; }
  void setWriteLimit(size_t value) { _write_limit = value; }

 private:
  std::deque<TimedByte> _rx{};
  std::vector<uint8_t> _tx{};
  std::vector<uint32_t> _tx_write_times{};
  std::vector<Operation> _operations{};
  uint32_t _write_cost_us = 0;
  uint32_t _flush_cost_us = 0;
  size_t _write_limit = static_cast<size_t>(-1);
};

uint16_t crc16(const std::vector<uint8_t>& bytes) {
  uint16_t value = 0xFFFF;
  for (uint8_t b : bytes) {
    value ^= static_cast<uint16_t>(b);
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const bool lsb = (value & 0x0001U) != 0U;
      value >>= 1;
      if (lsb) {
        value ^= 0xA001U;
      }
    }
  }
  return value;
}

std::vector<uint8_t> makeFrame(uint8_t unit, uint8_t fc, std::initializer_list<uint8_t> payload) {
  std::vector<uint8_t> out;
  out.reserve(4 + payload.size());
  out.push_back(unit);
  out.push_back(fc);
  out.insert(out.end(), payload.begin(), payload.end());
  const uint16_t crc = crc16(out);
  out.push_back(lowByte(crc));
  out.push_back(highByte(crc));
  return out;
}

std::vector<uint8_t> makeRegisterReadResponse(uint8_t unit, uint16_t quantity) {
  std::vector<uint8_t> out;
  const uint16_t byteCount = static_cast<uint16_t>(quantity * 2U);
  out.reserve(static_cast<std::size_t>(byteCount) + 5U);
  out.push_back(unit);
  out.push_back(0x03);
  out.push_back(static_cast<uint8_t>(byteCount));
  for (uint16_t i = 0; i < quantity; ++i) {
    out.push_back(highByte(i));
    out.push_back(lowByte(i));
  }
  const uint16_t crc = crc16(out);
  out.push_back(lowByte(crc));
  out.push_back(highByte(crc));
  return out;
}

void setReadReq(ModbusADU& req, uint8_t unit, uint8_t fc, uint16_t start, uint16_t quantity) {
  req.setUnitId(unit);
  req.setFunctionCode(fc);
  req.setDataRegister(0, start);
  req.setDataRegister(2, quantity);
  req.setDataLen(4);
}

void setBroadcastWriteMultiRegReq(ModbusADU& req, uint16_t start, uint16_t quantity, uint16_t value0) {
  req.setUnitId(0);
  req.setFunctionCode(0x10);
  req.setDataRegister(0, start);
  req.setDataRegister(2, quantity);
  req.data[4] = static_cast<uint8_t>(quantity * 2U);
  req.setDataRegister(5, value0);
  req.setDataLen(static_cast<uint16_t>(5U + req.data[4]));
}

void setTargetedBroadcastWriteMultiRegReq(ModbusADU& req,
                                          uint8_t target,
                                          uint16_t start,
                                          uint16_t quantity,
                                          uint16_t value0) {
  req.setUnitId(0);
  req.setFunctionCode(0x45);
  req.data[0] = target;
  req.data[1] = 0x10;
  req.data[2] = highByte(start);
  req.data[3] = lowByte(start);
  req.data[4] = highByte(quantity);
  req.data[5] = lowByte(quantity);
  req.data[6] = static_cast<uint8_t>(quantity * 2U);
  for (uint16_t i = 0; i < quantity; ++i) {
    const uint16_t value = static_cast<uint16_t>(value0 + i);
    req.data[7U + i * 2U] = highByte(value);
    req.data[8U + i * 2U] = lowByte(value);
  }
  req.setDataLen(static_cast<uint16_t>(7U + quantity * 2U));
}

void resetTestClock() {
  arduino_test::reset_time(0);
  arduino_test::set_micros_step(8);
  arduino_test::yield_call_count() = 0;
  arduino_test::clear_io_events();
#if MBUS_RTU_PLATFORM_TRACE
  capturedPlatformTrace().clear();
#endif
}

uint32_t streamOperationCount(
    const std::vector<ScriptedStream::Operation>& operations,
    ScriptedStream::Operation expected) {
  uint32_t count = 0;
  for (ScriptedStream::Operation operation : operations) {
    if (operation == expected) {
      ++count;
    }
  }
  return count;
}

std::vector<arduino_test::IoEvent> txLifecycleEvents(int8_t dePin) {
  std::vector<arduino_test::IoEvent> events;
  bool transmitting = false;
  for (const arduino_test::IoEvent& event : arduino_test::io_events()) {
    if (event.operation == arduino_test::IoOperation::DigitalWrite &&
        event.arg0 == dePin) {
      if (event.arg1 == HIGH) {
        transmitting = true;
      }
      if (transmitting) {
        events.push_back(event);
      }
      continue;
    }
    if (!transmitting) {
      continue;
    }
    if (event.operation == arduino_test::IoOperation::StreamWrite ||
        event.operation == arduino_test::IoOperation::StreamDrain ||
        event.operation == arduino_test::IoOperation::DelayMilliseconds ||
        event.operation == arduino_test::IoOperation::DelayMicroseconds) {
      events.push_back(event);
    }
  }
  return events;
}

void assertTxLifecycle(const std::vector<arduino_test::IoEvent>& events,
                       int8_t dePin,
                       uint16_t attemptedBytes,
                       uint16_t writtenBytes,
                       uint32_t writeCostUs,
                       uint32_t drainCostUs,
                       uint32_t postDelayUs) {
  TEST_ASSERT_EQUAL_UINT32(5U, events.size());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::DigitalWrite),
      static_cast<uint8_t>(events[0].operation));
  TEST_ASSERT_EQUAL_INT8(dePin, events[0].arg0);
  TEST_ASSERT_EQUAL_INT(HIGH, events[0].arg1);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::StreamWrite),
      static_cast<uint8_t>(events[1].operation));
  TEST_ASSERT_EQUAL_UINT16(attemptedBytes, events[1].arg0);
  TEST_ASSERT_EQUAL_UINT16(writtenBytes, events[1].arg1);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::StreamDrain),
      static_cast<uint8_t>(events[2].operation));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::DelayMicroseconds),
      static_cast<uint8_t>(events[3].operation));
  TEST_ASSERT_EQUAL_UINT32(postDelayUs, events[3].arg0);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::DigitalWrite),
      static_cast<uint8_t>(events[4].operation));
  TEST_ASSERT_EQUAL_INT8(dePin, events[4].arg0);
  TEST_ASSERT_EQUAL_INT(LOW, events[4].arg1);

  const uint64_t traceTimestampCostUs =
      MBUS_RTU_PLATFORM_TRACE ? arduino_test::micros_step_us() : 0U;
  TEST_ASSERT_EQUAL_UINT64(
      traceTimestampCostUs,
      events[1].timestamp_us - events[0].timestamp_us);
  TEST_ASSERT_EQUAL_UINT64(
      writeCostUs,
      events[2].timestamp_us - events[1].timestamp_us);
  TEST_ASSERT_EQUAL_UINT64(
      drainCostUs,
      events[3].timestamp_us - events[2].timestamp_us);
  TEST_ASSERT_EQUAL_UINT64(
      postDelayUs,
      events[4].timestamp_us - events[3].timestamp_us);
}

void test_high_baud_uses_spec_fixed_timers_without_changing_8n1_wire_time() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  TEST_ASSERT_EQUAL_UINT32(40U, comm._charTimeUs);
  TEST_ASSERT_EQUAL_UINT32(750U, comm._charTimeout);
  TEST_ASSERT_EQUAL_UINT32(1750U, comm._frameTimeout);
  TEST_ASSERT_EQUAL_UINT32(204264U, comm._maxFrameReceiveUs);
}

void test_transaction_budget_is_wrap_safe_and_preserves_receive_tail() {
  const RxTimingBudget normal =
      make_rx_timing_budget(250000U, 8000U, 5563676U);
  TEST_ASSERT_EQUAL_UINT32(250000U, normal.read_timeout_us);
  TEST_ASSERT_EQUAL_UINT32(5821676U, normal.active_escape_us);

  const RxTimingBudget extreme =
      make_rx_timing_budget(0xFFFFFFFFUL, 8000U, 5563676U);
  TEST_ASSERT_EQUAL_UINT32(kMaxWrapSafeRxIntervalUs,
                           extreme.active_escape_us);
  TEST_ASSERT_EQUAL_UINT32(
      kMaxWrapSafeRxIntervalUs - 8000U - 5563676U,
      extreme.read_timeout_us);

  const uint32_t startUs = 0xFFFFFF00UL;
  TEST_ASSERT_FALSE(rx_interval_elapsed(startUs, 0x0000002BUL, 300U));
  TEST_ASSERT_TRUE(rx_interval_elapsed(startUs, 0x0000002CUL, 300U));

  const uint32_t boundaryStartUs = 0xFFF00000UL;
  TEST_ASSERT_FALSE(rx_interval_elapsed(
      boundaryStartUs,
      boundaryStartUs + normal.active_escape_us - 1U,
      normal.active_escape_us));
  TEST_ASSERT_TRUE(rx_interval_elapsed(
      boundaryStartUs,
      boundaryStartUs + normal.active_escape_us,
      normal.active_escape_us));

  // Current bytes retain their actual timestamp; slightly prefetched bytes are
  // clamped only to readAdu() entry; genuinely stale/ambiguous bytes use now.
  TEST_ASSERT_EQUAL_UINT32(
      10200U,
      normalize_rx_event_timestamp(10000U, 10500U, 10200U, 1000U));
  TEST_ASSERT_EQUAL_UINT32(
      10000U,
      normalize_rx_event_timestamp(10000U, 10500U, 9900U, 1000U));
  TEST_ASSERT_EQUAL_UINT32(
      2400000000UL,
      normalize_rx_event_timestamp(
          2400000000UL, 2400000000UL, 1000U, 1000000U));
}

// FC03 permits 125 registers, producing a 255-byte RTU response. The response
// may start just inside the application timeout but must not inherit that same
// timeout as a whole-frame deadline.
void test_maximum_fc03_response_completes_after_response_start_timeout() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(4);

  ModbusADU req;
  setReadReq(req, 0x28, 0x03, 0, 125);
  const auto response = makeRegisterReadResponse(0x28, 125);
  TEST_ASSERT_EQUAL_UINT16(255U, response.size());
  const uint32_t firstUs = arduino_test::now_us() + 3900U;
  serial.pushBytes(response, firstUs, 40U);

  const ModbusRTUCommError err = comm.readAdu(req);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, err);
  TEST_ASSERT_EQUAL_UINT16(response.size(), req.getRtuLen());
  TEST_ASSERT_TRUE(comm.debugInfo().last_read_total_us > 4000U);
}

// At 1200 baud the maximum FC03 response lasts over two seconds on wire. Let
// its first byte arrive inside late grace to exercise the longest legal path:
// response-start timeout + late window + the complete frame envelope.
void test_low_baud_maximum_fc03_response_survives_late_start_and_full_frame() {
  resetTestClock();
  arduino_test::set_micros_step(100U);
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(1200, SERIAL_8N1);
  comm.setTimeout(4);

  ModbusADU req;
  setReadReq(req, 0x28, 0x03, 0, 125);
  const auto response = makeRegisterReadResponse(0x28, 125);
  TEST_ASSERT_EQUAL_UINT16(255U, response.size());
  TEST_ASSERT_EQUAL_UINT32(8334U, comm._charTimeUs);
  TEST_ASSERT_EQUAL_UINT32(5563676U, comm._maxFrameReceiveUs);

  const uint32_t lateGraceUs = comm._computeLateGraceUs();
  const uint32_t firstUs = arduino_test::now_us() + 4000U + lateGraceUs - 500U;
  serial.pushBytes(response, firstUs, comm._charTimeUs);
  const uint64_t readStartUs = arduino_test::clock_us();

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, comm.readAdu(req));
  TEST_ASSERT_EQUAL_UINT16(response.size(), req.getRtuLen());
  TEST_ASSERT_TRUE(comm.debugInfo().late_match_after_timeout_count >= 1U);
  TEST_ASSERT_GREATER_THAN_UINT64(2000000ULL,
                                  arduino_test::clock_us() - readStartUs);
  TEST_ASSERT_LESS_THAN_UINT64(
      static_cast<uint64_t>(4000U + lateGraceUs + comm._maxFrameReceiveUs),
      arduino_test::clock_us() - readStartUs);
}

// Exercise the same maximum response at 300 baud, below the usual Modbus RTU
// deployment floor. This is deliberately conservative: if a platform accepts
// that baud, the transaction cap still leaves the entire legal frame intact.
void test_300_baud_maximum_fc03_response_is_not_preempted() {
  resetTestClock();
  arduino_test::set_micros_step(500U);
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(300, SERIAL_8N1);
  comm.setTimeout(4);

  ModbusADU req;
  setReadReq(req, 0x29, 0x03, 0, 125);
  const auto response = makeRegisterReadResponse(0x29, 125);
  TEST_ASSERT_EQUAL_UINT16(255U, response.size());
  TEST_ASSERT_EQUAL_UINT32(33334U, comm._charTimeUs);
  TEST_ASSERT_EQUAL_UINT32(22253676U, comm._maxFrameReceiveUs);

  const uint32_t lateGraceUs = comm._computeLateGraceUs();
  const uint32_t firstUs =
      arduino_test::now_us() + 4000U + lateGraceUs - 2000U;
  serial.pushBytes(response, firstUs, comm._charTimeUs);
  const uint64_t readStartUs = arduino_test::clock_us();

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, comm.readAdu(req));
  TEST_ASSERT_EQUAL_UINT16(response.size(), req.getRtuLen());
  TEST_ASSERT_TRUE(comm.debugInfo().late_match_after_timeout_count >= 1U);
  TEST_ASSERT_GREATER_THAN_UINT64(8000000ULL,
                                  arduino_test::clock_us() - readStartUs);
  TEST_ASSERT_LESS_THAN_UINT64(
      static_cast<uint64_t>(4000U + lateGraceUs + comm._maxFrameReceiveUs),
      arduino_test::clock_us() - readStartUs);
  TEST_ASSERT_EQUAL_UINT32(0U, comm.debugInfo().transaction_escape_count);
}

// The fixed high-baud T1.5 recommendation is also the parser boundary: an
// inter-character interval equal to 750 us remains valid.
void test_high_baud_t15_boundary_is_inclusive() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(4);

  ModbusADU req;
  setReadReq(req, 0x29, 0x03, 0, 1);
  const auto response = makeFrame(0x29, 0x03, {0x02, 0x12, 0x34});
  uint32_t timestampUs = 100U;
  for (std::size_t i = 0; i < response.size(); ++i) {
    if (i > 0U) {
      timestampUs += (i == 3U) ? 750U : 40U;
    }
    TEST_ASSERT_TRUE(comm._pushRxByte(response[i], timestampUs));
  }
  arduino_test::advance_us(10000U);

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, comm.readAdu(req));
}

void test_high_baud_gap_above_t15_is_rejected_then_resyncs() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(20);

  ModbusADU req;
  setReadReq(req, 0x2A, 0x03, 0, 1);
  const auto bad = makeFrame(0x2A, 0x03, {0x02, 0x12, 0x34});
  const auto good = makeFrame(0x2A, 0x03, {0x02, 0x56, 0x78});

  uint32_t timestampUs = 100U;
  for (std::size_t i = 0; i < bad.size(); ++i) {
    if (i > 0U) {
      timestampUs += (i == 3U) ? 751U : 40U;
    }
    TEST_ASSERT_TRUE(comm._pushRxByte(bad[i], timestampUs));
  }
  timestampUs += 1750U;
  for (uint8_t value : good) {
    TEST_ASSERT_TRUE(comm._pushRxByte(value, timestampUs));
    timestampUs += 40U;
  }
  arduino_test::advance_us(10000U);

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, comm.readAdu(req));
  TEST_ASSERT_TRUE(comm.debugInfo().recovered_t15_count >= 1U);
}

// E4: Stray frame (unexpected slave) must be discarded while expected reply is
// still accepted in the same transaction.
void test_stray_then_expected_frame_is_accepted() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(20);

  ModbusADU req;
  setReadReq(req, 0x0A, 0x03, 0, 2);

  const auto stray = makeFrame(0x0B, 0x03, {0x04, 0x00, 0x10, 0x00, 0x11});
  const auto good = makeFrame(0x0A, 0x03, {0x04, 0x00, 0x21, 0x00, 0x22});
  const uint32_t base = arduino_test::now_us() + 200;
  serial.pushBytes(stray, base, 40);
  serial.pushBytes(good, base + static_cast<uint32_t>(stray.size() * 40), 40);

  const ModbusRTUCommError err = comm.readAdu(req);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, err);
  TEST_ASSERT_EQUAL_UINT8(0x0A, req.getUnitId());
  TEST_ASSERT_EQUAL_UINT8(0x03, req.getFunctionCode());
  TEST_ASSERT_EQUAL_UINT16(9, req.getRtuLen());
  TEST_ASSERT_TRUE(comm.debugInfo().recovered_stray_count >= 1);
}

// E1: Late response should still be accepted inside grace, preventing a false
// terminal timeout/retry collision.
void test_late_response_within_grace_is_accepted() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(2);

  ModbusADU req;
  setReadReq(req, 0x12, 0x04, 0, 2);

  const auto good = makeFrame(0x12, 0x04, {0x04, 0x00, 0x31, 0x00, 0x32});
  const uint32_t base = arduino_test::now_us() + 3000;
  serial.pushBytes(good, base, 35);

  const ModbusRTUCommError err = comm.readAdu(req);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, err);
  TEST_ASSERT_TRUE(comm.debugInfo().recovered_late_count >= 1);
  TEST_ASSERT_TRUE(comm.debugInfo().late_match_after_timeout_count >= 1);
}

// Baseline timeout path when no frame ever arrives.
void test_timeout_without_response_reports_timeout() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(1);

  ModbusADU req;
  setReadReq(req, 0x15, 0x03, 0, 1);
  const ModbusRTUCommError err = comm.readAdu(req);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_TIMEOUT, err);
  TEST_ASSERT_TRUE(comm.debugInfo().timeout_count >= 1);
  TEST_ASSERT_TRUE(comm.debugInfo().timeout_no_rx_count >= 1);
  TEST_ASSERT_EQUAL_UINT32(0, comm.debugInfo().timeout_with_rx_count);
  TEST_ASSERT_EQUAL_UINT32(0, comm.debugInfo().timeout_with_parse_err_count);
}

// A stale marker more than 2^31 us old used to compare as newer than the
// transaction start and arm an RX deadline in the next micros() cycle.
void test_stale_rx_marker_after_signed_half_range_does_not_activate_rx() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(1);

  comm._lastRxByteUs.store(1000U, MBUS_MEM_RELAXED);
  arduino_test::reset_time(0x80010000ULL);
  arduino_test::set_micros_step(100000U);
  const uint64_t startUs = arduino_test::clock_us();

  ModbusADU req;
  setReadReq(req, 0x15, 0x03, 0, 1);
  const ModbusRTUCommError err = comm.readAdu(req);

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_TIMEOUT, err);
  TEST_ASSERT_EQUAL_UINT32(1, comm.debugInfo().timeout_no_rx_count);
  TEST_ASSERT_EQUAL_UINT32(0, comm.debugInfo().timeout_with_rx_count);
  TEST_ASSERT_LESS_THAN_UINT64(5000000ULL, arduino_test::clock_us() - startUs);
}

// The same retained marker can look recent after a complete micros() wrap.
// An empty ring must still be classified as a no-RX timeout.
void test_stale_rx_marker_after_full_wrap_does_not_activate_rx() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(1);

  comm._lastRxByteUs.store(2000U, MBUS_MEM_RELAXED);
  arduino_test::reset_time(0x100000000ULL + 1000ULL);

  ModbusADU req;
  setReadReq(req, 0x15, 0x03, 0, 1);
  const ModbusRTUCommError err = comm.readAdu(req);

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_TIMEOUT, err);
  TEST_ASSERT_EQUAL_UINT32(1, comm.debugInfo().timeout_no_rx_count);
  TEST_ASSERT_EQUAL_UINT32(0, comm.debugInfo().timeout_with_rx_count);
}

// Defensive direct-read case: maintained master callers normally drain in
// writeAdu() first, but a nonstandard caller can expose an entry retained for
// over forty minutes. Its wrapped timestamp must not seed future RX deadlines.
void test_stale_queued_fragment_after_forty_minutes_is_bounded() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(1);

  TEST_ASSERT_TRUE(comm._pushRxByte(0x16U, 1000U));
  arduino_test::reset_time(2400000000ULL);
  arduino_test::set_micros_step(1000U);
  const uint64_t startUs = arduino_test::clock_us();

  ModbusADU req;
  setReadReq(req, 0x16, 0x03, 0, 1);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_FRAME_ERROR, comm.readAdu(req));
  TEST_ASSERT_EQUAL_UINT16(0U, req.getRtuLen());
  TEST_ASSERT_LESS_THAN_UINT64(
      static_cast<uint64_t>(comm._maxFrameReceiveUs),
      arduino_test::clock_us() - startUs);
  TEST_ASSERT_EQUAL_UINT32(0U, comm.debugInfo().transaction_escape_count);
}

// The same stale-entry defense must survive a full micros() wrap and retain
// the existing frame-over-CRC terminal precedence for a corrupt complete ADU.
void test_stale_corrupt_frame_after_full_wrap_keeps_error_precedence() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(1);

  std::vector<uint8_t> bad = makeFrame(0x31, 0x03, {0x02, 0x12, 0x34});
  bad.back() ^= 0x01U;
  uint32_t eventUs = 1000U;
  for (uint8_t value : bad) {
    TEST_ASSERT_TRUE(comm._pushRxByte(value, eventUs));
    eventUs += 40U;
  }

  arduino_test::reset_time(0x100000000ULL + 2400000000ULL);
  arduino_test::set_micros_step(1000U);
  const uint64_t startUs = arduino_test::clock_us();
  ModbusADU req;
  setReadReq(req, 0x31, 0x03, 0, 1);

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_FRAME_ERROR, comm.readAdu(req));
  TEST_ASSERT_EQUAL_UINT16(0U, req.getRtuLen());
  TEST_ASSERT_LESS_THAN_UINT64(
      static_cast<uint64_t>(comm._maxFrameReceiveUs),
      arduino_test::clock_us() - startUs);
  TEST_ASSERT_EQUAL_UINT32(0U, comm.debugInfo().transaction_escape_count);
}

// Force only the defensive budget seam to expire, starting immediately before
// micros() wrap. This proves the cap itself is elapsed-time based and observable
// without changing normal timeout/error semantics.
void test_transaction_escape_is_observable_across_micros_wrap() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(1);
  comm._maxFrameReceiveUs = 0U;

  arduino_test::reset_time(0xFFFFFF00ULL);
  arduino_test::set_micros_step(100U);
  ModbusADU req;
  setReadReq(req, 0x15, 0x03, 0, 1);

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_TIMEOUT, comm.readAdu(req));
  TEST_ASSERT_EQUAL_UINT32(1U, comm.debugInfo().transaction_escape_count);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ModbusRTUComm::RxTxnState::LATE_WINDOW),
      comm.debugInfo().last_transaction_escape_state);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(
      3000U, comm.debugInfo().last_transaction_escape_us);
  TEST_ASSERT_EQUAL_UINT32(
      3000U, comm.debugInfo().last_transaction_escape_budget_us);
  TEST_ASSERT_EQUAL_UINT16(0U,
                           comm.debugInfo().last_transaction_escape_rx_count);
}

// A busy line can keep producing complete but unrelated candidates forever.
// The transaction cap must be checked after each classified non-match, not only
// when the extractor reports no work. Keep enough real Stream/extractor input
// queued that the escape is observably taken before the candidate burst ends.
void test_continuous_stray_frames_cannot_evade_transaction_escape() {
  resetTestClock();
  arduino_test::set_micros_step(50U);
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(1);
  comm._maxFrameReceiveUs = 12000U;

  const auto stray = makeFrame(0x16, 0x03, {0x02, 0x12, 0x34});
  const uint32_t readyUs = arduino_test::now_us();
  for (uint8_t frameIndex = 0; frameIndex < 36U; ++frameIndex) {
    serial.pushBytes(stray, readyUs, 0U);
  }

  ModbusADU req;
  setReadReq(req, 0x15, 0x03, 0, 1);
  const uint64_t startUs = arduino_test::clock_us();
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_TIMEOUT, comm.readAdu(req));

  TEST_ASSERT_EQUAL_UINT32(1U, comm.debugInfo().transaction_escape_count);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ModbusRTUComm::RxTxnState::PROCESS_FRAME),
      comm.debugInfo().last_transaction_escape_state);
  TEST_ASSERT_GREATER_THAN_UINT16(
      0U, comm.debugInfo().last_transaction_escape_rx_count);
  TEST_ASSERT_TRUE(comm.debugInfo().recovered_stray_count >= 1U);
  TEST_ASSERT_TRUE(comm._rxEmpty());
  TEST_ASSERT_LESS_THAN_UINT64(
      static_cast<uint64_t>(15000U + MBUS_RTU_DRAIN_ESCAPE_US),
      arduino_test::clock_us() - startUs);
}

// The same busy-candidate escape must not flatten accumulated parser failures
// into a timeout. Corrupt matching-shaped frames retain frame-over-CRC terminal
// precedence even when H, rather than a quiet line, ends classification.
void test_continuous_corrupt_frames_escape_with_error_precedence() {
  resetTestClock();
  arduino_test::set_micros_step(50U);
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(1);
  comm._maxFrameReceiveUs = 12000U;

  std::vector<uint8_t> corrupt =
      makeFrame(0x15, 0x03, {0x02, 0x12, 0x34});
  corrupt.back() ^= 0x01U;
  const uint32_t readyUs = arduino_test::now_us();
  // Seed an abandoned fragment before the corrupt burst. The parser observes
  // both framing and CRC damage, so the final result must retain frame > CRC >
  // timeout precedence when H fires.
  TEST_ASSERT_TRUE(comm._pushRxByte(0x15U, readyUs - 10000U));
  for (uint8_t frameIndex = 0; frameIndex < 36U; ++frameIndex) {
    serial.pushBytes(corrupt, readyUs, 0U);
  }

  ModbusADU req;
  setReadReq(req, 0x15, 0x03, 0, 1);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_FRAME_ERROR, comm.readAdu(req));

  TEST_ASSERT_EQUAL_UINT16(0U, req.getRtuLen());
  TEST_ASSERT_EQUAL_UINT32(1U, comm.debugInfo().transaction_escape_count);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ModbusRTUComm::RxTxnState::PROCESS_FRAME),
      comm.debugInfo().last_transaction_escape_state);
  TEST_ASSERT_GREATER_THAN_UINT16(
      0U, comm.debugInfo().last_transaction_escape_rx_count);
  TEST_ASSERT_TRUE(comm.debugInfo().frame_err_count >= 1U);
  TEST_ASSERT_TRUE(comm._rxEmpty());
}

// PROCESS_FRAME classifies an already-extracted candidate before consulting H.
// Advance the fake clock past H between extraction and classification and prove
// a valid matching response still succeeds instead of becoming an escape.
void test_matching_candidate_at_transaction_boundary_wins_over_escape() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(1);
  comm._maxFrameReceiveUs = 0U;

  ModbusADU req;
  setReadReq(req, 0x15, 0x03, 0, 1);
  const auto response = makeFrame(0x15, 0x03, {0x02, 0x12, 0x34});
  uint32_t timestampUs = arduino_test::now_us();
  for (uint8_t value : response) {
    TEST_ASSERT_TRUE(comm._pushRxByte(value, timestampUs));
    timestampUs += 40U;
  }
  arduino_test::set_micros_step(2000U);

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, comm.readAdu(req));
  TEST_ASSERT_EQUAL_UINT16(response.size(), req.getRtuLen());
  TEST_ASSERT_EQUAL_UINT32(0U, comm.debugInfo().transaction_escape_count);
}

// Standard Master ordering drains old ingress before TX. Keep this separate
// from the direct-read defense so a stale ring cannot silently become a common
// transaction-cap path.
void test_write_then_read_drains_stale_ring_without_transaction_escape() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(1);

  TEST_ASSERT_TRUE(comm._pushRxByte(0x15U, 1000U));
  arduino_test::reset_time(2400000000ULL);
  arduino_test::set_micros_step(100U);

  ModbusADU req;
  setReadReq(req, 0x15, 0x03, 0, 1);
  TEST_ASSERT_TRUE(comm.writeAdu(req));
  TEST_ASSERT_TRUE(comm._rxEmpty());
  const uint64_t readStartUs = arduino_test::clock_us();
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_TIMEOUT, comm.readAdu(req));

  TEST_ASSERT_EQUAL_UINT32(1U, comm.debugInfo().timeout_no_rx_count);
  TEST_ASSERT_EQUAL_UINT32(0U, comm.debugInfo().timeout_with_rx_count);
  TEST_ASSERT_EQUAL_UINT32(0U, comm.debugInfo().transaction_escape_count);
  TEST_ASSERT_LESS_THAN_UINT64(20000ULL,
                               arduino_test::clock_us() - readStartUs);
}

// A real first byte just before the first-byte deadline must still extend the
// receive window for the remainder of the frame.
void test_partial_response_near_first_byte_timeout_completes() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(10);

  ModbusADU req;
  setReadReq(req, 0x16, 0x03, 0, 1);
  const auto good = makeFrame(0x16, 0x03, {0x02, 0x12, 0x34});
  const std::vector<uint8_t> firstByte{good.front()};
  const std::vector<uint8_t> remainder(good.begin() + 1, good.end());
  const uint32_t firstUs = arduino_test::now_us() + 9500U;
  serial.pushBytes(firstByte, firstUs, 0);
  serial.pushBytes(remainder, firstUs + 900U, 50U);

  const ModbusRTUCommError err = comm.readAdu(req);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, err);
  TEST_ASSERT_EQUAL_UINT16(good.size(), req.getRtuLen());
}

// Even a fragment shorter than the four-byte RTU minimum is discarded after
// T3.5; it must not fall through to the 250 ms hard drain escape.
void test_incomplete_response_stays_within_bounded_wait_and_drain() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(2);

  ModbusADU req;
  setReadReq(req, 0x16, 0x03, 0, 1);
  comm._pushRxByte(0x16, arduino_test::now_us());
  const uint64_t startUs = arduino_test::clock_us();

  const ModbusRTUCommError err = comm.readAdu(req);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_FRAME_ERROR, err);
  TEST_ASSERT_LESS_OR_EQUAL_UINT64(
      static_cast<uint64_t>(comm._frameTimeout) + 1000ULL,
      arduino_test::clock_us() - startUs);
  TEST_ASSERT_EQUAL_UINT32(0U, comm.debugInfo().drain_escape_count);
}

// Timeout path when RX activity exists but no valid matching frame is completed.
void test_timeout_with_rx_activity_is_classified() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(2);

  ModbusADU req;
  setReadReq(req, 0x16, 0x03, 0, 1);

  // Valid but unrelated frame starts RX activity; request should still timeout.
  const uint32_t base = arduino_test::now_us() + 300;
  const auto stray = makeFrame(0x17, 0x03, {0x02, 0x00, 0x01});
  serial.pushBytes(stray, base, 35);

  const ModbusRTUCommError err = comm.readAdu(req);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_TIMEOUT, err);
  TEST_ASSERT_TRUE(comm.debugInfo().timeout_count >= 1);
  TEST_ASSERT_TRUE(comm.debugInfo().timeout_with_rx_count >= 1);
  TEST_ASSERT_EQUAL_UINT32(0, comm.debugInfo().timeout_no_rx_count);
  TEST_ASSERT_EQUAL_UINT32(0, comm.debugInfo().timeout_with_parse_err_count);
}

// E3/C2: Inter-character gap > T1.5 invalidates partial frame, then parser
// must resync and accept later clean frame.
void test_t15_violation_drops_bad_segment_then_resyncs() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(20);

  ModbusADU req;
  setReadReq(req, 0x22, 0x03, 0, 2);

  const std::vector<uint8_t> partial = {0x22, 0x03, 0x04};
  const auto good = makeFrame(0x22, 0x03, {0x04, 0x00, 0x41, 0x00, 0x42});
  const uint32_t base = arduino_test::now_us() + 200;
  serial.pushBytes(partial, base, 60);
  serial.pushBytes(good, base + 5000, 40);

  const ModbusRTUCommError err = comm.readAdu(req);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, err);
  TEST_ASSERT_TRUE(comm.debugInfo().recovered_t15_count >= 1);
}

// C1: CRC-corrupt frame should not poison stream; next valid frame must parse.
void test_crc_error_resyncs_and_accepts_next_frame() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(20);

  ModbusADU req;
  setReadReq(req, 0x31, 0x03, 0, 2);

  auto bad = makeFrame(0x31, 0x03, {0x04, 0x00, 0x10, 0x00, 0x11});
  bad[3] ^= 0x01; // break CRC while preserving frame shape.
  const auto good = makeFrame(0x31, 0x03, {0x04, 0x00, 0x51, 0x00, 0x52});
  const uint32_t base = arduino_test::now_us() + 200;
  serial.pushBytes(bad, base, 35);
  serial.pushBytes(good, base + static_cast<uint32_t>(bad.size() * 35 + 4000), 35);

  const ModbusRTUCommError err = comm.readAdu(req);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, err);
  TEST_ASSERT_TRUE(comm.debugInfo().recovered_parse_count >= 1);
}

// E5: Duplicate prior response in short window should be ignored for the new
// transaction; next matching response must still succeed.
void test_duplicate_within_window_is_dropped_for_next_tx() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(20);

  ModbusADU req1;
  setReadReq(req1, 0x44, 0x03, 0, 1);
  TEST_ASSERT_TRUE(comm.writeAdu(req1));
  const auto resp1 = makeFrame(0x44, 0x03, {0x02, 0x00, 0x61});
  uint32_t t = arduino_test::now_us() + 200;
  serial.pushBytes(resp1, t, 35);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, comm.readAdu(req1));

  ModbusADU req2;
  setReadReq(req2, 0x44, 0x03, 0, 2);
  TEST_ASSERT_TRUE(comm.writeAdu(req2));
  const auto resp2 = makeFrame(0x44, 0x03, {0x04, 0x00, 0x71, 0x00, 0x72});
  t = arduino_test::now_us() + 200;
  serial.pushBytes(resp1, t, 35); // duplicate of prior successful response
  serial.pushBytes(resp2, t + static_cast<uint32_t>(resp1.size() * 35 + 200), 35);

  const ModbusRTUCommError err2 = comm.readAdu(req2);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, err2);
  TEST_ASSERT_EQUAL_UINT8(0x04, req2.data[0]); // accepted frame is the current response
  TEST_ASSERT_TRUE(comm.debugInfo().recovered_duplicate_count >= 1);
}

// Pre-buffered RX bytes can legitimately have timestamps that predate the readAdu()
// start when an async ingress path is active. Debug timing must clamp those
// timestamps instead of underflowing to near-UINT32_MAX.
void test_prefetched_frame_timestamps_do_not_underflow_debug_metrics() {
  resetTestClock();
  arduino_test::advance_us(10000);
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(20);

  ModbusADU req;
  setReadReq(req, 0x55, 0x03, 0, 2);

  const auto good = makeFrame(0x55, 0x03, {0x04, 0x00, 0x81, 0x00, 0x82});
  uint32_t ts = arduino_test::now_us() - 300;
  for (uint8_t b : good) {
    comm._pushRxByte(b, ts);
    ts += 50;
  }

  const ModbusRTUCommError err = comm.readAdu(req);
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, err);
  TEST_ASSERT_LESS_THAN_UINT32(1000U, comm.debugInfo().last_wait_first_us);
  TEST_ASSERT_LESS_THAN_UINT32(1000U, comm.debugInfo().last_read_total_us);
}

// B1: Broadcast writes are no-reply transactions, but still impose a turnaround
// gate before next TX. Validate gate intent via internal scheduling boundary.
void test_broadcast_write_enforces_turnaround_gap() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(9600);
  comm.setTimeout(20);

  ModbusADU bcastReq;
  setBroadcastWriteMultiRegReq(bcastReq, 0, 1, 0x1234);
  TEST_ASSERT_TRUE(comm.writeAdu(bcastReq));
  const uint32_t now = arduino_test::now_us();
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(
      comm._frameTimeout, comm._remainingTxGateUs(now));
}

// A visual page is a no-response FC69 transaction. The next clean-line drain
// must preserve, rather than replace, the holdoff armed by that first page.
void test_consecutive_fc69_pages_preserve_first_no_reply_gate() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  ModbusADU first;
  setTargetedBroadcastWriteMultiRegReq(first, 20, 0, 32, 0x1000);
  TEST_ASSERT_TRUE(comm.writeAdu(first));
  const uint32_t firstGate =
      comm._txGateStartedUs + comm._txGateDurationUs;

  ModbusADU second;
  setTargetedBroadcastWriteMultiRegReq(second, 20, 32, 32, 0x2000);
  TEST_ASSERT_TRUE(comm.writeAdu(second));
  TEST_ASSERT_TRUE(static_cast<int32_t>(
      comm.debugInfo().last_tx_start_us - firstGate) >= 0);
}

// A response-bearing operation queued behind FC69 is governed by the same
// retained boundary; strict traffic must not overtake the slave processing gap.
void test_strict_request_after_fc69_preserves_no_reply_gate() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  ModbusADU visual;
  setTargetedBroadcastWriteMultiRegReq(visual, 20, 0, 32, 0x3000);
  TEST_ASSERT_TRUE(comm.writeAdu(visual));
  const uint32_t visualGate =
      comm._txGateStartedUs + comm._txGateDurationUs;

  ModbusADU strict;
  setReadReq(strict, 34, 0x03, 0, 1);
  TEST_ASSERT_TRUE(comm.writeAdu(strict));
  TEST_ASSERT_TRUE(static_cast<int32_t>(
      comm.debugInfo().last_tx_start_us - visualGate) >= 0);
}

void test_tx_success_preserves_driver_write_drain_delay_order() {
  resetTestClock();
  ScriptedStream serial;
  const int8_t dePin = 7;
  ModbusRTUComm comm(serial, dePin, 8);
  comm.begin(250000, SERIAL_8N1);

  serial.clear();
  arduino_test::clear_io_events();
  serial.setWriteCostUs(120U);
  serial.setFlushCostUs(80U);
  comm._postDelay = 37U;
  ModbusADU request;
  setBroadcastWriteMultiRegReq(request, 0x0020, 1, 0xA55A);

  TEST_ASSERT_TRUE(comm.writeAdu(request));
  TEST_ASSERT_EQUAL_UINT16(request.getRtuLen(), serial.txBytes().size());
  assertTxLifecycle(
      txLifecycleEvents(dePin),
      dePin,
      request.getRtuLen(),
      request.getRtuLen(),
      120U,
      80U,
      37U);
}

void test_partial_tx_still_drains_delays_and_restores_driver_low() {
  resetTestClock();
  ScriptedStream serial;
  const int8_t dePin = 9;
  ModbusRTUComm comm(serial, dePin, 10);
  comm.begin(250000, SERIAL_8N1);

  serial.clear();
  arduino_test::clear_io_events();
  serial.setWriteLimit(3U);
  serial.setWriteCostUs(45U);
  serial.setFlushCostUs(25U);
  comm._postDelay = 19U;
  ModbusADU request;
  setBroadcastWriteMultiRegReq(request, 0x0030, 1, 0x5AA5);

  TEST_ASSERT_FALSE(comm.writeAdu(request));
  TEST_ASSERT_EQUAL_UINT32(3U, serial.txBytes().size());
  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_FRAME_ERROR,
                          comm.debugInfo().last_err);
  assertTxLifecycle(
      txLifecycleEvents(dePin),
      dePin,
      request.getRtuLen(),
      3U,
      45U,
      25U,
      19U);
}

void test_begin_initializes_de_and_re_receive_safe_but_tx_toggles_only_de() {
  resetTestClock();
  ScriptedStream serial;
  const int8_t dePin = 11;
  const int8_t rePin = 12;
  ModbusRTUComm comm(serial, dePin, rePin);
  comm.begin(250000, SERIAL_8N1);

  const std::vector<arduino_test::IoEvent>& beginEvents =
      arduino_test::io_events();
  TEST_ASSERT_EQUAL_UINT32(4U, beginEvents.size());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::PinMode),
      static_cast<uint8_t>(beginEvents[0].operation));
  TEST_ASSERT_EQUAL_INT8(dePin, beginEvents[0].arg0);
  TEST_ASSERT_EQUAL_INT(OUTPUT, beginEvents[0].arg1);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::DigitalWrite),
      static_cast<uint8_t>(beginEvents[1].operation));
  TEST_ASSERT_EQUAL_INT8(dePin, beginEvents[1].arg0);
  TEST_ASSERT_EQUAL_INT(LOW, beginEvents[1].arg1);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::PinMode),
      static_cast<uint8_t>(beginEvents[2].operation));
  TEST_ASSERT_EQUAL_INT8(rePin, beginEvents[2].arg0);
  TEST_ASSERT_EQUAL_INT(OUTPUT, beginEvents[2].arg1);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(arduino_test::IoOperation::DigitalWrite),
      static_cast<uint8_t>(beginEvents[3].operation));
  TEST_ASSERT_EQUAL_INT8(rePin, beginEvents[3].arg0);
  TEST_ASSERT_EQUAL_INT(LOW, beginEvents[3].arg1);

  serial.clear();
  arduino_test::clear_io_events();
  ModbusADU request;
  setReadReq(request, 0x11, 0x03, 0, 1);
  TEST_ASSERT_TRUE(comm.writeAdu(request));

  uint32_t deTransitions = 0;
  uint32_t reTransitions = 0;
  for (const arduino_test::IoEvent& event : arduino_test::io_events()) {
    if (event.operation != arduino_test::IoOperation::DigitalWrite) {
      continue;
    }
    if (event.arg0 == dePin) {
      ++deTransitions;
    } else if (event.arg0 == rePin) {
      ++reTransitions;
    }
  }
  TEST_ASSERT_EQUAL_UINT32(2U, deTransitions);
  TEST_ASSERT_EQUAL_UINT32(0U, reTransitions);
}

void test_one_shot_pre_gap_zero_maximum_and_consumption_are_stable() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  // Isolate the one-shot API from begin()'s initial clean-line holdoff.
  comm._replaceTxGate(arduino_test::now_us(), 0U);
  const uint32_t zeroStarted = comm._txGateStartedUs;
  const uint32_t zeroDuration = comm._txGateDurationUs;
  comm.setPreTxGapUsOnce(0U);
  TEST_ASSERT_EQUAL_UINT32(zeroStarted, comm._txGateStartedUs);
  TEST_ASSERT_EQUAL_UINT32(zeroDuration, comm._txGateDurationUs);

  comm.setPreTxGapUsOnce(4000U);
  const uint32_t firstGateStarted = comm._txGateStartedUs;
  const uint32_t firstGateDuration = comm._txGateDurationUs;
  comm.setPreTxGapUsOnce(1000U);
  TEST_ASSERT_EQUAL_UINT32(firstGateStarted, comm._txGateStartedUs);
  TEST_ASSERT_EQUAL_UINT32(firstGateDuration, comm._txGateDurationUs);
  comm.setPreTxGapUsOnce(7000U);
  const uint32_t maximumGate =
      comm._txGateStartedUs + comm._txGateDurationUs;
  TEST_ASSERT_GREATER_THAN_UINT32(firstGateDuration,
                                  comm._txGateDurationUs);

  ModbusADU first;
  setReadReq(first, 0x11, 0x03, 0, 1);
  TEST_ASSERT_TRUE(comm.writeAdu(first));
  TEST_ASSERT_TRUE(static_cast<int32_t>(
      comm.debugInfo().last_tx_start_us - maximumGate) >= 0);

  // writeAdu() replaces the consumed pre-gap with the normal next-frame gate.
  const uint32_t afterFirst = arduino_test::now_us();
  const uint32_t firstScheduledGap = comm._remainingTxGateUs(afterFirst);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(comm._frameTimeout + 64U,
                                  firstScheduledGap);

  arduino_test::advance_us(firstScheduledGap + 16U);
  ModbusADU second;
  setReadReq(second, 0x12, 0x03, 0, 1);
  TEST_ASSERT_TRUE(comm.writeAdu(second));
  const uint32_t secondScheduledGap = comm._remainingTxGateUs(
      arduino_test::now_us());
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(comm._frameTimeout + 64U,
                                  secondScheduledGap);
}

void test_one_shot_post_gap_uses_maximum_and_is_consumed_after_success() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  comm.setPostTxGapUsOnce(0U);
  TEST_ASSERT_EQUAL_UINT32(0U, comm._oneShotPostGapUs);
  comm.setPostTxGapUsOnce(3000U);
  comm.setPostTxGapUsOnce(1000U);
  comm.setPostTxGapUsOnce(5000U);
  TEST_ASSERT_EQUAL_UINT32(5000U, comm._oneShotPostGapUs);

  ModbusADU first;
  setReadReq(first, 0x11, 0x03, 0, 1);
  TEST_ASSERT_TRUE(comm.writeAdu(first));
  TEST_ASSERT_EQUAL_UINT32(0U, comm._oneShotPostGapUs);
  const uint32_t postGap = comm._remainingTxGateUs(arduino_test::now_us());
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(comm._frameTimeout + 5000U - 64U,
                                     postGap);

  arduino_test::advance_us(postGap + 16U);
  ModbusADU second;
  setReadReq(second, 0x12, 0x03, 0, 1);
  TEST_ASSERT_TRUE(comm.writeAdu(second));
  const uint32_t normalGap = comm._remainingTxGateUs(arduino_test::now_us());
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(comm._frameTimeout + 64U, normalGap);
}

void test_partial_write_consumes_post_gap_but_preserves_failure_holdoff() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  serial.setWriteLimit(3U);
  comm.setPostTxGapUsOnce(4000U);

  ModbusADU request;
  setReadReq(request, 0x11, 0x03, 0, 1);
  TEST_ASSERT_FALSE(comm.writeAdu(request));
  TEST_ASSERT_EQUAL_UINT32(0U, comm._oneShotPostGapUs);
  const uint32_t failureGap = comm._remainingTxGateUs(arduino_test::now_us());
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(comm._frameTimeout + 4000U - 64U,
                                     failureGap);

  arduino_test::advance_us(failureGap + 16U);
  serial.setWriteLimit(static_cast<size_t>(-1));
  TEST_ASSERT_TRUE(comm.writeAdu(request));
  const uint32_t normalGap = comm._remainingTxGateUs(arduino_test::now_us());
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(comm._frameTimeout + 64U, normalGap);
}

void test_tx_gate_elapsed_interval_is_exact_across_wrap_and_long_idle() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);

  const uint32_t startUs = 0xFFFFFFF0UL;
  comm._replaceTxGate(startUs, 100U);
  TEST_ASSERT_EQUAL_UINT32(100U, comm._remainingTxGateUs(startUs));
  TEST_ASSERT_EQUAL_UINT32(1U, comm._remainingTxGateUs(startUs + 99U));
  TEST_ASSERT_EQUAL_UINT32(0U, comm._remainingTxGateUs(startUs + 100U));

  // These are the old signed-deadline danger zone and the observed idle age.
  // A bounded elapsed interval is already expired at each boundary.
  comm._replaceTxGate(1000U, 6000U);
  TEST_ASSERT_EQUAL_UINT32(
      0U, comm._remainingTxGateUs(1000U + 0x80000000UL));
  TEST_ASSERT_EQUAL_UINT32(
      0U, comm._remainingTxGateUs(1000U + 3209007000UL));
  TEST_ASSERT_EQUAL_UINT32(
      0U,
      comm._remainingTxGateUs(
          static_cast<uint32_t>(1000ULL + 0xFFFFFFFFULL)));

  TEST_ASSERT_EQUAL_UINT32(
      0x7FFFFFFFUL, comm._boundedTxGateUs(0x100000000ULL));
}

void test_retained_backend_byte_cannot_revive_dormant_tx_gate() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  // RX threading is disabled in the production profile, so a byte may remain
  // in the serial backend until the next write.  Place the old gate just below
  // a full micros() wrap: the historical signed deadline implementation would
  // ingest this byte, refresh the idle watchdog, then sleep for about 50 ms.
  serial.clear();
  serial.pushBytes({0xA5U}, arduino_test::now_us(), 0U);
  arduino_test::advance_us(0xFFFFFFFFULL - 50000ULL);
  arduino_test::clear_io_events();

  ModbusADU request;
  setReadReq(request, 0x11, 0x03, 0x006B, 0x0003);
  const uint64_t startUs = arduino_test::clock_us();
  TEST_ASSERT_TRUE(comm.writeAdu(request));
  const uint64_t elapsedUs = arduino_test::clock_us() - startUs;

  TEST_ASSERT_LESS_THAN_UINT64(10000ULL, elapsedUs);
  TEST_ASSERT_LESS_THAN_UINT32(1000U, comm.debugInfo().last_tx_wait_us);
  uint32_t millisecondSleeps = 0U;
  for (const arduino_test::IoEvent& event : arduino_test::io_events()) {
    if (event.operation == arduino_test::IoOperation::DelayMilliseconds) {
      ++millisecondSleeps;
    }
  }
  TEST_ASSERT_EQUAL_UINT32(0U, millisecondSleeps);
}

void test_retained_backend_byte_cannot_revive_gate_after_full_micros_wrap() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  // A large public one-shot makes the exact full-wrap alias observable. The
  // pre-ingress idle watchdog must clear it before this delayed backend byte
  // refreshes last-traffic state.
  comm.setPreTxGapUsOnce(60000000UL);
  serial.clear();
  serial.pushBytes({0x5AU}, arduino_test::now_us(), 0U);
  arduino_test::advance_us(0x100000000ULL);

  ModbusADU request;
  setReadReq(request, 0x11, 0x03, 0x006B, 0x0003);
  const uint64_t startUs = arduino_test::clock_us();
  TEST_ASSERT_TRUE(comm.writeAdu(request));
  const uint64_t elapsedUs = arduino_test::clock_us() - startUs;

  TEST_ASSERT_LESS_THAN_UINT64(10000ULL, elapsedUs);
  TEST_ASSERT_LESS_THAN_UINT32(1000U, comm.debugInfo().last_tx_wait_us);
}

void test_rx_thread_byte_cannot_revive_gate_after_full_micros_wrap() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  // The RX-thread profile can ingest a byte and refresh lastTraffic before the
  // transport thread reaches _drainToIdle(). Gate age therefore has to be
  // independent of traffic age, not merely checked before polling Stream.
  comm.setPreTxGapUsOnce(60000000UL);
  arduino_test::advance_us(0x100000000ULL);
  TEST_ASSERT_TRUE(
      comm._pushRxByte(0x5AU, arduino_test::now_us()));

  ModbusADU request;
  setReadReq(request, 0x11, 0x03, 0x006B, 0x0003);
  const uint64_t startUs = arduino_test::clock_us();
  TEST_ASSERT_TRUE(comm.writeAdu(request));
  const uint64_t elapsedUs = arduino_test::clock_us() - startUs;

  TEST_ASSERT_LESS_THAN_UINT64(10000ULL, elapsedUs);
  TEST_ASSERT_LESS_THAN_UINT32(1000U, comm.debugInfo().last_tx_wait_us);
}

void test_rx_ring_capacity_overflow_drop_and_index_wrap_are_bounded() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  TEST_ASSERT_EQUAL_UINT32(256U, MBUS_RTU_RX_RING_SIZE);
  TEST_ASSERT_EQUAL_UINT32(8U, sizeof(ModbusRTUComm::RxEntry));
  TEST_ASSERT_EQUAL_UINT32(
      2048U, sizeof(comm._rxRing));

  // One slot distinguishes full from empty, so a 256-entry ring holds 255.
  for (uint16_t i = 0; i < 255U; ++i) {
    TEST_ASSERT_TRUE(comm._pushRxByte(static_cast<uint8_t>(i), 1000U + i));
  }
  TEST_ASSERT_EQUAL_UINT16(255U, comm._rxCount());
  TEST_ASSERT_FALSE(comm._pushRxByte(0xEEU, 2000U));
  TEST_ASSERT_EQUAL_UINT32(1U,
      comm._rxOverflowCount.load(MBUS_MEM_RELAXED));

  uint8_t byte = 0;
  uint32_t timestamp = 0;
  for (uint16_t i = 0; i < 200U; ++i) {
    TEST_ASSERT_TRUE(comm._popRxByte(byte, timestamp));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(i), byte);
    TEST_ASSERT_EQUAL_UINT32(1000U + i, timestamp);
  }

  // Refill across index zero until full again, then prove overflow accounting.
  for (uint16_t i = 0; i < 200U; ++i) {
    TEST_ASSERT_TRUE(comm._pushRxByte(
        static_cast<uint8_t>(0x80U + i), 3000U + i));
  }
  TEST_ASSERT_EQUAL_UINT16(255U, comm._rxCount());
  TEST_ASSERT_TRUE(comm._rxHead.load(MBUS_MEM_RELAXED) <
                   comm._rxTail.load(MBUS_MEM_RELAXED));
  TEST_ASSERT_FALSE(comm._pushRxByte(0xEFU, 4000U));
  TEST_ASSERT_EQUAL_UINT32(2U,
      comm._rxOverflowCount.load(MBUS_MEM_RELAXED));

  // Drop the 55 retained old entries. The next pop crosses the physical end.
  comm._dropRxBytes(55U);
  TEST_ASSERT_EQUAL_UINT16(200U, comm._rxCount());
  for (uint16_t i = 0; i < 200U; ++i) {
    TEST_ASSERT_TRUE(comm._popRxByte(byte, timestamp));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(0x80U + i), byte);
    TEST_ASSERT_EQUAL_UINT32(3000U + i, timestamp);
  }
  TEST_ASSERT_TRUE(comm._rxEmpty());
  TEST_ASSERT_EQUAL_UINT32(
      comm._rxHead.load(MBUS_MEM_RELAXED),
      comm._rxTail.load(MBUS_MEM_RELAXED));
}

// The transport does not require a transceiver-local echo before
// reporting a completed write. Freeze both that policy and the exact FC03
// request bytes so a change cannot add an echo-read loop or perturb CRC order.
void test_exact_fc03_tx_bytes_succeed_without_local_echo() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  serial.clear();
  ModbusADU request;
  setReadReq(request, 0x11, 0x03, 0x006B, 0x0003);

  TEST_ASSERT_TRUE(comm.writeAdu(request));
  const uint8_t expected[] = {
      0x11, 0x03, 0x00, 0x6B, 0x00, 0x03, 0x76, 0x87};
  TEST_ASSERT_EQUAL_UINT32(sizeof(expected), serial.txBytes().size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, serial.txBytes().data(), sizeof(expected));
}

void test_exact_fc69_no_reply_bytes_and_crc_are_stable() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  serial.clear();
  ModbusADU request;
  setTargetedBroadcastWriteMultiRegReq(request, 20, 0, 1, 0x1234);

  TEST_ASSERT_TRUE(comm.writeAdu(request));
  const uint8_t expected[] = {
      0x00, 0x45, 0x14, 0x10, 0x00, 0x00, 0x00,
      0x01, 0x02, 0x12, 0x34, 0x47, 0x67};
  TEST_ASSERT_EQUAL_UINT32(sizeof(expected), serial.txBytes().size());
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, serial.txBytes().data(), sizeof(expected));
}

// CRC resynchronization drops one byte, then classifies the remaining fragment
// as framing damage. Preserve the established frame-over-CRC terminal
// precedence and, more importantly, never expose the corrupt bytes to callers.
void test_crc_corruption_uses_frame_precedence_and_clears_output_adu() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(3);

  ModbusADU request;
  setReadReq(request, 0x31, 0x03, 0, 1);
  std::vector<uint8_t> bad = makeFrame(0x31, 0x03, {0x02, 0x12, 0x34});
  bad.back() ^= 0x01U;
  serial.pushBytes(bad, arduino_test::now_us() + 100U, 40U);

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_FRAME_ERROR, comm.readAdu(request));
  TEST_ASSERT_EQUAL_UINT16(0U, request.getRtuLen());
}

// Logical fake-clock budget: an empty-line write performs one T3.5 drain and
// then the TX lifecycle. This is intentionally an upper bound rather than a
// wall-clock benchmark, making it deterministic while still catching an extra
// frame wait, retry loop, or hidden millisecond sleep in the hot path.
void test_empty_line_tx_stays_within_single_frame_wait_budget() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  serial.clear();
  ModbusADU request;
  setReadReq(request, 0x11, 0x03, 0x006B, 0x0003);
  const uint64_t startUs = arduino_test::clock_us();

  TEST_ASSERT_TRUE(comm.writeAdu(request));
  const uint64_t elapsedUs = arduino_test::clock_us() - startUs;
  TEST_ASSERT_GREATER_OR_EQUAL_UINT64(comm._frameTimeout, elapsedUs);
  TEST_ASSERT_LESS_OR_EQUAL_UINT64(
      static_cast<uint64_t>(comm._frameTimeout) + 1000ULL, elapsedUs);
}

// A prefetched normal response has a fixed transport hot path. The safety
// checks are arithmetic-only and must not add Stream calls or cooperative
// yields to that path.
void test_normal_prefetched_read_preserves_serial_calls_and_yield_count() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(4);

  ModbusADU request;
  setReadReq(request, 0x32, 0x03, 0, 1);
  const auto response = makeFrame(0x32, 0x03, {0x02, 0xBE, 0xEF});
  uint32_t timestampUs = arduino_test::now_us() + 100U;
  for (uint8_t value : response) {
    TEST_ASSERT_TRUE(comm._pushRxByte(value, timestampUs));
    timestampUs += 40U;
  }
  arduino_test::advance_us(5000U);
  serial.clear();
  arduino_test::yield_call_count() = 0U;

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, comm.readAdu(request));
  const std::vector<ScriptedStream::Operation>& operations = serial.operations();
  TEST_ASSERT_EQUAL_UINT32(
      4U, streamOperationCount(operations, ScriptedStream::Operation::Available));
  TEST_ASSERT_EQUAL_UINT32(
      0U, streamOperationCount(operations, ScriptedStream::Operation::Read));
  TEST_ASSERT_EQUAL_UINT32(
      0U, streamOperationCount(operations, ScriptedStream::Operation::Write));
  TEST_ASSERT_EQUAL_UINT32(
      0U, streamOperationCount(operations, ScriptedStream::Operation::Flush));
  TEST_ASSERT_EQUAL_UINT32(0U, arduino_test::yield_call_count());
  TEST_ASSERT_EQUAL_UINT32(0U, comm.debugInfo().transaction_escape_count);
}

#if MBUS_RTU_PLATFORM_TRACE
void test_platform_ingress_delegation_preserves_byte_call_order_and_trace() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  serial.clear();
  capturedPlatformTrace().clear();
  const auto frame = makeFrame(0x31, 0x03, {0x02, 0x12, 0x34});
  serial.pushBytes(frame, arduino_test::now_us(), 0U);

  comm._ingestPoll();

  const std::vector<ScriptedStream::Operation>& operations = serial.operations();
  TEST_ASSERT_EQUAL_UINT32(frame.size() * 2U + 1U, operations.size());
  for (std::size_t i = 0; i < frame.size(); ++i) {
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(ScriptedStream::Operation::Available),
        static_cast<uint8_t>(operations[i * 2U]));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(ScriptedStream::Operation::Read),
        static_cast<uint8_t>(operations[i * 2U + 1U]));
  }
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(ScriptedStream::Operation::Available),
      static_cast<uint8_t>(operations.back()));

  const std::vector<CapturedPlatformTrace>& trace = capturedPlatformTrace();
  TEST_ASSERT_EQUAL_UINT32(frame.size(), trace.size());
  for (std::size_t i = 0; i < frame.size(); ++i) {
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(modbus_rtu::platform::TraceEvent::IngressByte),
        trace[i].event);
    TEST_ASSERT_EQUAL_UINT8(frame[i], trace[i].detail);
    if (i > 0U) {
      TEST_ASSERT_TRUE(trace[i].timestamp_us >= trace[i - 1U].timestamp_us);
    }
  }
}

void test_platform_tx_delegation_preserves_write_flush_and_trace_order() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);

  serial.clear();
  serial.setWriteCostUs(120U);
  serial.setFlushCostUs(80U);
  capturedPlatformTrace().clear();
  ModbusADU request;
  setBroadcastWriteMultiRegReq(request, 0x0020, 1, 0xA55A);
  TEST_ASSERT_TRUE(comm.writeAdu(request));

  std::size_t writeCount = 0;
  std::size_t flushCount = 0;
  std::size_t writeIndex = 0;
  std::size_t flushIndex = 0;
  const std::vector<ScriptedStream::Operation>& operations = serial.operations();
  for (std::size_t i = 0; i < operations.size(); ++i) {
    if (operations[i] == ScriptedStream::Operation::Write) {
      ++writeCount;
      writeIndex = i;
    } else if (operations[i] == ScriptedStream::Operation::Flush) {
      ++flushCount;
      flushIndex = i;
    }
  }
  TEST_ASSERT_EQUAL_UINT32(1U, writeCount);
  TEST_ASSERT_EQUAL_UINT32(1U, flushCount);
  TEST_ASSERT_EQUAL_UINT32(writeIndex + 1U, flushIndex);

  const std::vector<CapturedPlatformTrace>& trace = capturedPlatformTrace();
  TEST_ASSERT_EQUAL_UINT32(3U, trace.size());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(modbus_rtu::platform::TraceEvent::TxGateOpen),
      trace[0].event);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(modbus_rtu::platform::TraceEvent::TxWriteStarted),
      trace[1].event);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(modbus_rtu::platform::TraceEvent::TxWriteFinished),
      trace[2].event);
  TEST_ASSERT_EQUAL_UINT16(request.getRtuLen(), trace[1].detail);
  TEST_ASSERT_EQUAL_UINT32(208U, trace[2].timestamp_us - trace[1].timestamp_us);
  TEST_ASSERT_EQUAL_UINT32(
      (static_cast<uint32_t>(request.getRtuLen()) << 1U) | 1U,
      trace[2].detail);
}

void test_success_state_trace_preserves_transaction_transition_order() {
  resetTestClock();
  ScriptedStream serial;
  ModbusRTUComm comm(serial);
  comm.begin(250000, SERIAL_8N1);
  comm.setTimeout(4);

  ModbusADU request;
  setReadReq(request, 0x32, 0x03, 0, 1);
  const auto response = makeFrame(0x32, 0x03, {0x02, 0xBE, 0xEF});
  uint32_t timestampUs = arduino_test::now_us() + 100U;
  for (uint8_t value : response) {
    TEST_ASSERT_TRUE(comm._pushRxByte(value, timestampUs));
    timestampUs += 40U;
  }
  arduino_test::advance_us(1000U);
  capturedPlatformTrace().clear();

  TEST_ASSERT_EQUAL_UINT8(MODBUS_RTU_COMM_SUCCESS, comm.readAdu(request));

  std::vector<uint32_t> transitions;
  for (const CapturedPlatformTrace& record : capturedPlatformTrace()) {
    if (record.event == static_cast<uint8_t>(
                            modbus_rtu::platform::TraceEvent::RxStateTransition)) {
      transitions.push_back(record.detail);
    }
  }
  TEST_ASSERT_EQUAL_UINT32(4U, transitions.size());
  TEST_ASSERT_EQUAL_HEX32(0x0001U, transitions[0]);  // WAIT_FIRST -> RX_ACTIVE
  TEST_ASSERT_EQUAL_HEX32(0x0102U, transitions[1]);  // RX_ACTIVE -> PROCESS_FRAME
  TEST_ASSERT_EQUAL_HEX32(0x0204U, transitions[2]);  // PROCESS_FRAME -> DRAIN
  TEST_ASSERT_EQUAL_HEX32(0x0405U, transitions[3]);  // DRAIN -> DONE
}
#endif

} // namespace

void run_modbus_rtu_transport_tests() {
  // Ordered to move from framing fundamentals to edge conditions.
  RUN_TEST(test_high_baud_uses_spec_fixed_timers_without_changing_8n1_wire_time);
  RUN_TEST(test_transaction_budget_is_wrap_safe_and_preserves_receive_tail);
  RUN_TEST(test_maximum_fc03_response_completes_after_response_start_timeout);
  RUN_TEST(test_low_baud_maximum_fc03_response_survives_late_start_and_full_frame);
  RUN_TEST(test_300_baud_maximum_fc03_response_is_not_preempted);
  RUN_TEST(test_high_baud_t15_boundary_is_inclusive);
  RUN_TEST(test_high_baud_gap_above_t15_is_rejected_then_resyncs);
  RUN_TEST(test_stray_then_expected_frame_is_accepted);
  RUN_TEST(test_late_response_within_grace_is_accepted);
  RUN_TEST(test_timeout_without_response_reports_timeout);
  RUN_TEST(test_stale_rx_marker_after_signed_half_range_does_not_activate_rx);
  RUN_TEST(test_stale_rx_marker_after_full_wrap_does_not_activate_rx);
  RUN_TEST(test_stale_queued_fragment_after_forty_minutes_is_bounded);
  RUN_TEST(test_stale_corrupt_frame_after_full_wrap_keeps_error_precedence);
  RUN_TEST(test_transaction_escape_is_observable_across_micros_wrap);
  RUN_TEST(test_continuous_stray_frames_cannot_evade_transaction_escape);
  RUN_TEST(test_continuous_corrupt_frames_escape_with_error_precedence);
  RUN_TEST(test_matching_candidate_at_transaction_boundary_wins_over_escape);
  RUN_TEST(test_write_then_read_drains_stale_ring_without_transaction_escape);
  RUN_TEST(test_partial_response_near_first_byte_timeout_completes);
  RUN_TEST(test_incomplete_response_stays_within_bounded_wait_and_drain);
  RUN_TEST(test_timeout_with_rx_activity_is_classified);
  RUN_TEST(test_t15_violation_drops_bad_segment_then_resyncs);
  RUN_TEST(test_crc_error_resyncs_and_accepts_next_frame);
  RUN_TEST(test_duplicate_within_window_is_dropped_for_next_tx);
  RUN_TEST(test_prefetched_frame_timestamps_do_not_underflow_debug_metrics);
  RUN_TEST(test_broadcast_write_enforces_turnaround_gap);
  RUN_TEST(test_consecutive_fc69_pages_preserve_first_no_reply_gate);
  RUN_TEST(test_strict_request_after_fc69_preserves_no_reply_gate);
  RUN_TEST(test_tx_success_preserves_driver_write_drain_delay_order);
  RUN_TEST(test_partial_tx_still_drains_delays_and_restores_driver_low);
  RUN_TEST(test_begin_initializes_de_and_re_receive_safe_but_tx_toggles_only_de);
  RUN_TEST(test_one_shot_pre_gap_zero_maximum_and_consumption_are_stable);
  RUN_TEST(test_one_shot_post_gap_uses_maximum_and_is_consumed_after_success);
  RUN_TEST(test_partial_write_consumes_post_gap_but_preserves_failure_holdoff);
  RUN_TEST(test_tx_gate_elapsed_interval_is_exact_across_wrap_and_long_idle);
  RUN_TEST(test_retained_backend_byte_cannot_revive_dormant_tx_gate);
  RUN_TEST(test_retained_backend_byte_cannot_revive_gate_after_full_micros_wrap);
  RUN_TEST(test_rx_thread_byte_cannot_revive_gate_after_full_micros_wrap);
  RUN_TEST(test_rx_ring_capacity_overflow_drop_and_index_wrap_are_bounded);
  RUN_TEST(test_exact_fc03_tx_bytes_succeed_without_local_echo);
  RUN_TEST(test_exact_fc69_no_reply_bytes_and_crc_are_stable);
  RUN_TEST(test_crc_corruption_uses_frame_precedence_and_clears_output_adu);
  RUN_TEST(test_empty_line_tx_stays_within_single_frame_wait_budget);
  RUN_TEST(test_normal_prefetched_read_preserves_serial_calls_and_yield_count);
#if MBUS_RTU_PLATFORM_TRACE
  RUN_TEST(test_platform_ingress_delegation_preserves_byte_call_order_and_trace);
  RUN_TEST(test_platform_tx_delegation_preserves_write_flush_and_trace_order);
  RUN_TEST(test_success_state_trace_preserves_transaction_transition_order);
#endif
}

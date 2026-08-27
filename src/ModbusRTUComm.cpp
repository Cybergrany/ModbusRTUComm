// Modbus RTU transport implementation.
// Derived-source attribution is covered by the repository MIT license.
#include "ModbusRTUComm.h"
#include "ModbusRTUPlatformBinding.h"

#include <assert.h>
#include <string.h>

namespace {
typedef MBUS_RTU_PLATFORM_TYPE MbusPlatform;
typedef modbus_rtu::platform::TraceEvent MbusPlatformTraceEvent;
typedef modbus_rtu::platform::TraceRecord MbusPlatformTraceRecord;
}

#if MBUS_RTU_STATE_ASSERTS
  #define MBUS_RTU_ASSERT(expr) assert(expr)
#else
  #define MBUS_RTU_ASSERT(expr) ((void)0)
#endif

template<typename T>
static inline T max_val(T a, T b) {
  return (a > b) ? a : b;
}

template<typename T>
static inline T min_val(T a, T b) {
  return (a < b) ? a : b;
}

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline bool timestamp_not_before(uint32_t referenceUs, uint32_t sampleUs) {
  return static_cast<int32_t>(sampleUs - referenceUs) >= 0;
}

// Every absolute deadline in the RX state machine is compared through a signed
// 32-bit delta. Keep one transaction strictly inside that unambiguous half of
// the micros() range, including the late-response and maximum-frame windows.
static const uint32_t kMaxWrapSafeRxIntervalUs = 0x7FFFFFFFUL;
static const uint32_t kMaxWrapSafeTxIntervalUs = 0x7FFFFFFFUL;

struct RxTimingBudget {
  uint32_t read_timeout_us;
  uint32_t active_escape_us;

  RxTimingBudget(uint32_t readTimeoutUs, uint32_t activeEscapeUs)
      : read_timeout_us(readTimeoutUs), active_escape_us(activeEscapeUs) {}
};

static inline RxTimingBudget make_rx_timing_budget(uint32_t requestedReadUs,
                                                   uint32_t lateGraceUs,
                                                   uint32_t maxFrameReceiveUs) {
  // H = response-start wait + late acceptance + the conservative maximum RTU
  // frame envelope (which already includes T3.5). DRAIN is bounded separately.
  const uint64_t receiveTailWide =
      static_cast<uint64_t>(lateGraceUs) +
      static_cast<uint64_t>(maxFrameReceiveUs);
  const uint32_t receiveTailUs =
      receiveTailWide > kMaxWrapSafeRxIntervalUs
          ? kMaxWrapSafeRxIntervalUs
          : static_cast<uint32_t>(receiveTailWide);
  const uint32_t maxReadUs = kMaxWrapSafeRxIntervalUs - receiveTailUs;
  const uint32_t effectiveReadUs = min_val(requestedReadUs, maxReadUs);
  return RxTimingBudget(effectiveReadUs, effectiveReadUs + receiveTailUs);
}

static inline bool rx_interval_elapsed(uint32_t startUs,
                                       uint32_t nowUs,
                                       uint32_t durationUs) {
  return static_cast<uint32_t>(nowUs - startUs) >= durationUs;
}

static inline uint32_t normalize_rx_event_timestamp(uint32_t transactionStartUs,
                                                    uint32_t observedNowUs,
                                                    uint32_t eventUs,
                                                    uint32_t maxTrustedAgeUs) {
  // Once an entry is older than the entire transaction window, its wrapped
  // uint32_t timestamp is ambiguous: signed ordering can mistake it for a
  // future byte and seed a deadline in the next micros() cycle. Keep parsing
  // the queued byte for compatibility, but anchor its receive windows at the
  // observation time. Genuine prefetched/current bytes retain their timestamp.
  if (static_cast<uint32_t>(observedNowUs - eventUs) > maxTrustedAgeUs) {
    return observedNowUs;
  }
  return timestamp_not_before(transactionStartUs, eventUs)
             ? eventUs
             : transactionStartUs;
}

static inline uint32_t elapsed_us_or_zero(uint32_t startUs, uint32_t endUs) {
  return timestamp_not_before(startUs, endUs) ? static_cast<uint32_t>(endUs - startUs) : 0U;
}

#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED
void mbusDiagSerialLock() {
  MbusPlatform::diagnosticsLock();
}

void mbusDiagSerialUnlock() {
  MbusPlatform::diagnosticsUnlock();
}

bool mbusDiagSerialTryLock() {
  return MbusPlatform::diagnosticsTryLock();
}

bool mbusDiagSerialCanWrite(size_t bytesHint) {
  return MbusPlatform::diagnosticsCanWrite(bytesHint);
}
#endif

ModbusRTUComm::ModbusRTUComm(Stream& serial, int8_t dePin, int8_t rePin)
    : _serial(serial), _dePin(dePin), _rePin(rePin) {}

ModbusRTUComm::~ModbusRTUComm() {
  _stopRxThread();
  _detachIngressAdapter();
}

void ModbusRTUComm::begin(unsigned long baud, uint32_t config) {
  _stopRxThread();
  _detachIngressAdapter();

  unsigned long bitsPerChar;
  switch (config) {
    case SERIAL_8E2:
    case SERIAL_8O2:
      bitsPerChar = 12;
      break;
    case SERIAL_8N2:
    case SERIAL_8E1:
    case SERIAL_8O1:
      bitsPerChar = 11;
      break;
    case SERIAL_8N1:
    default:
      bitsPerChar = 10;
      break;
  }

  _charTimeUs = (bitsPerChar * 1000000UL + baud - 1) / baud;

#if MBUS_RTU_TIMING_MODE == MBUS_RTU_TIMING_SPEC_FIXED_GT19200
  if (baud > 19200UL) {
    _charTimeout = MBUS_RTU_T15_FIXED_GT19200_US;
    _frameTimeout = MBUS_RTU_T35_FIXED_GT19200_US;
  } else {
    _charTimeout = (3UL * _charTimeUs + 1UL) / 2UL;
    _frameTimeout = (7UL * _charTimeUs + 1UL) / 2UL;
  }
#else
  _charTimeout = (3UL * _charTimeUs + 1UL) / 2UL;
  _frameTimeout = (7UL * _charTimeUs + 1UL) / 2UL;
#endif

#if MBUS_RTU_T15_OVERRIDE_US > 0UL
  _charTimeout = MBUS_RTU_T15_OVERRIDE_US;
#elif MBUS_RTU_T15_DEFAULT_GT19200_US > 0UL
  if (baud > 19200UL) {
    _charTimeout = MBUS_RTU_T15_DEFAULT_GT19200_US;
  }
#endif

  // Bound a continuously active transaction without reusing the application
  // response-start timeout as a whole-frame deadline. The RTU guide defines a
  // maximum 256-byte frame and an 11-bit serial character (8E1, or 8N2 when
  // parity is not used). Legacy 8N1 links therefore receive the conservative
  // 11-bit budget without changing their wire format.
  const unsigned long budgetBitsPerChar = max_val(bitsPerChar, 11UL);
  const uint64_t budgetCharUs =
      (static_cast<uint64_t>(budgetBitsPerChar) * 1000000ULL + baud - 1ULL) /
      baud;
  const uint64_t maxFrameReceiveUsWide =
      (256ULL * budgetCharUs) +
      (255ULL * static_cast<uint64_t>(_charTimeout)) +
      static_cast<uint64_t>(_frameTimeout);
  _maxFrameReceiveUs = maxFrameReceiveUsWide > 0x7FFFFFFFULL
                           ? 0x7FFFFFFFUL
                           : static_cast<unsigned long>(maxFrameReceiveUsWide);
  yieldThreshold = max_val(YIELD_MIN_US, _charTimeUs / 4UL);

#if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI) || defined(ARDUINO_GIGA) || (defined(ARDUINO_NANO_RP2040_CONNECT) && defined(ARDUINO_ARCH_MBED))
  _postDelay = _charTimeUs + 2UL;
#endif

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
  _dbg.char_time_us = _charTimeUs;
  _dbg.char_timeout_us = _charTimeout;
  _dbg.frame_timeout_us = _frameTimeout;
  _dbg.post_delay_us = _postDelay;
  _dbg.extra_gap_us = _baseExtraGapUs;
#endif

  MbusPlatform::configureDriverPins(_dePin, _rePin);

  // Reset parser/ring state.
  _rxHead.store(0, MBUS_MEM_RELAXED);
  _rxTail.store(0, MBUS_MEM_RELAXED);
  _rxOverflowCount.store(0, MBUS_MEM_RELAXED);
  _rxReadableHint.store(false, MBUS_MEM_RELAXED);
  _lastRxByteUs.store(0, MBUS_MEM_RELAXED);
  _lastAcceptedLen = 0;
  _lastAcceptedUs = 0;
  _lastAcceptedTxSeq = 0;
  _txSeq = 0;
  _oneShotPostGapUs = 0;
  _timedOutPending = TimedOutPending{};
#if MBUS_RTU_RX_THREAD_STACK_WATERMARK_ACTIVE
  _rxThreadStackSizeBytes.store(0, MBUS_MEM_RELAXED);
  _rxThreadStackLastFreeBytes.store(0, MBUS_MEM_RELAXED);
  _rxThreadStackMinFreeBytes.store(0, MBUS_MEM_RELAXED);
  _rxThreadStackSamples.store(0, MBUS_MEM_RELAXED);
#endif

  // Drain transport bytes up to one frame timeout.
  unsigned long drainStartUs = MbusPlatform::microsNow();
  do {
    if (MbusPlatform::available(_serial) > 0) {
      (void)MbusPlatform::read(_serial);
      drainStartUs = MbusPlatform::microsNow();
    }
  } while ((MbusPlatform::microsNow() - drainStartUs) < _frameTimeout);

  const uint32_t txGateStartUs = MbusPlatform::microsNow();
  _replaceTxGate(
      txGateStartUs,
      static_cast<uint64_t>(_frameTimeout) + _baseExtraGapUs);
  _lastTrafficMs.store(MbusPlatform::millisNow(), MBUS_MEM_RELAXED);

  _attachIngressAdapter();
  _startRxThreadIfEnabled();
}

void ModbusRTUComm::setTimeout(unsigned long timeout) {
  _readTimeout = timeout;
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
  _dbg.read_timeout_ms = _readTimeout;
#endif
}

uint32_t ModbusRTUComm::_boundedTxGateUs(uint64_t requestedUs) {
  return requestedUs > kMaxWrapSafeTxIntervalUs
             ? kMaxWrapSafeTxIntervalUs
             : static_cast<uint32_t>(requestedUs);
}

uint32_t ModbusRTUComm::_remainingTxGateUs(uint32_t nowUs) const {
  const uint32_t durationUs = _txGateDurationUs;
  if (durationUs == 0U) {
    return 0U;
  }

  const uint32_t elapsedUs = nowUs - _txGateStartedUs;
  return elapsedUs >= durationUs ? 0U : durationUs - elapsedUs;
}

void ModbusRTUComm::_replaceTxGate(uint32_t nowUs, uint64_t durationUs) {
  _txGateStartedUs = nowUs;
  _txGateStartedMs = MbusPlatform::millisNow();
  _txGateDurationUs = _boundedTxGateUs(durationUs);
}

void ModbusRTUComm::_extendTxGate(uint32_t nowUs, uint64_t durationUs) {
  const uint32_t requestedUs = _boundedTxGateUs(durationUs);
  if (requestedUs > _remainingTxGateUs(nowUs)) {
    _replaceTxGate(nowUs, requestedUs);
  }
}

void ModbusRTUComm::_expireTxGateByCoarseAge(uint32_t nowMs) {
  const uint32_t durationUs = _txGateDurationUs;
  if (durationUs == 0U) {
    return;
  }

  // millis() has a much longer wrap period than micros(). It is used only as
  // a coarse proof that this bounded interval has elapsed, so the hot TX wait
  // loop retains its original micros-only cost and precision.
  const uint32_t durationMsCeil =
      (durationUs / 1000U) + ((durationUs % 1000U) != 0U ? 1U : 0U);
  if (static_cast<uint32_t>(nowMs - _txGateStartedMs) > durationMsCeil) {
    _replaceTxGate(MbusPlatform::microsNow(), 0U);
  }
}

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
void ModbusRTUComm::_syncRxThreadStackDiagDebug() const {
#if MBUS_RTU_RX_THREAD_STACK_WATERMARK_ACTIVE && MBUS_RTU_RX_THREAD_STACK_WATERMARK_DIAG
  const uint32_t sizeBytes = _rxThreadStackSizeBytes.load(MBUS_MEM_RELAXED);
  const uint32_t lastFreeBytes = _rxThreadStackLastFreeBytes.load(MBUS_MEM_RELAXED);
  const uint32_t minFreeBytes = _rxThreadStackMinFreeBytes.load(MBUS_MEM_RELAXED);
  const uint32_t sampleCount = _rxThreadStackSamples.load(MBUS_MEM_RELAXED);

  _dbg.rx_thread_stack_size_bytes = sizeBytes;
  _dbg.rx_thread_stack_last_free_bytes = lastFreeBytes;
  _dbg.rx_thread_stack_min_free_bytes = minFreeBytes;
  _dbg.rx_thread_stack_max_used_bytes = (sizeBytes >= minFreeBytes) ? (sizeBytes - minFreeBytes) : 0U;
  _dbg.rx_thread_stack_samples = sampleCount;
#endif
}

const ModbusRTUComm::DebugInfo& ModbusRTUComm::debugInfo() const {
  _syncRxThreadStackDiagDebug();
  return _dbg;
}
#endif

void ModbusRTUComm::setPreTxGapUsOnce(unsigned long extraGapUs) {
  if (extraGapUs == 0) {
    return;
  }
  const uint32_t nowUs = MbusPlatform::microsNow();
  _extendTxGate(nowUs, extraGapUs);
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
  _dbg.extra_gap_us = _baseExtraGapUs + extraGapUs;
#endif
}

void ModbusRTUComm::setPostTxGapUsOnce(unsigned long extraGapUs) {
  if (extraGapUs == 0) {
    return;
  }
  const uint32_t boundedGapUs = _boundedTxGateUs(extraGapUs);
  if (boundedGapUs > _oneShotPostGapUs) {
    _oneShotPostGapUs = boundedGapUs;
  }
}

void ModbusRTUComm::_attachIngressAdapter() {
  _rxCallbackAttached = false;
  _rxCallbackAttached = MbusPlatform::attachReadable(
      _serial,
      &ModbusRTUComm::_serialReadableThunk,
      this);
}

void ModbusRTUComm::_detachIngressAdapter() {
  MbusPlatform::detachReadable(_serial, _rxCallbackAttached);
}

void ModbusRTUComm::_startRxThreadIfEnabled() {
#if MBUS_RTU_ENABLE_RX_THREAD && (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
  if (!_rxCallbackAttached || _rxThreadRunning) {
    return;
  }
  const modbus_rtu::platform::EventTaskConfig taskConfig(
      modbus_rtu::platform::TaskPriority::AboveNormal,
      MBUS_RTU_RX_THREAD_STACK_BYTES,
      "mbus-rx");
  if (!MbusPlatform::startEventTask(
          _rxThreadState,
          taskConfig,
          &ModbusRTUComm::_rxThreadThunk,
          this)) {
    return;
  }
  _rxThreadRunning = true;
  // attachReadable() precedes task creation. If its callback observed an edge
  // before the backend published a signalable handle, _wakeRxThread() could
  // not queue that edge. Acquire the callback's release-store and replay it
  // once startup has succeeded so edge-only input cannot wait for another byte.
  MbusPlatform::replayPendingEvent(
      _rxThreadState,
      _rxReadableHint.load(MBUS_MEM_ACQUIRE));
#else
  _rxThreadRunning = false;
#endif
}

void ModbusRTUComm::_stopRxThread() {
#if MBUS_RTU_ENABLE_RX_THREAD && (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
  if (!_rxThreadRunning || !_rxThreadState) {
    return;
  }

  MbusPlatform::stopEventTask(_rxThreadState);
  _rxThreadRunning = false;
#endif
}

void ModbusRTUComm::_wakeRxThread() {
#if MBUS_RTU_ENABLE_RX_THREAD && (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
  // The backend publishes the task handle before Thread::start(). A readable
  // callback in that narrow start window must still queue its semaphore token;
  // waiting for the caller's _rxThreadRunning bookkeeping would drop the wake.
  if (_rxThreadState) {
    MbusPlatform::signalEvent(_rxThreadState);
  }
#endif
}

void ModbusRTUComm::_onReadableSignal() {
#if MBUS_RTU_PLATFORM_TRACE
  MbusPlatform::trace(MbusPlatformTraceRecord(
      MbusPlatformTraceEvent::ReadableSignalled,
      MbusPlatform::microsNow()));
#endif
  _rxReadableHint.store(true, MBUS_MEM_RELEASE);
#if MBUS_RTU_PLATFORM_TRACE
  MbusPlatform::trace(MbusPlatformTraceRecord(
      MbusPlatformTraceEvent::EventTaskWakeRequested,
      MbusPlatform::microsNow()));
#endif
  _wakeRxThread();
}

void ModbusRTUComm::_serialReadableThunk(void* ctx) {
  if (!ctx) {
    return;
  }
  static_cast<ModbusRTUComm*>(ctx)->_onReadableSignal();
}

#if MBUS_RTU_ENABLE_RX_THREAD && (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
void ModbusRTUComm::_rxThreadThunk(void* ctx) {
  if (!ctx) {
    return;
  }
  static_cast<ModbusRTUComm*>(ctx)->_rxThreadMain();
}
#endif

void ModbusRTUComm::_rxThreadMain() {
#if MBUS_RTU_ENABLE_RX_THREAD && (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
  if (!_rxThreadState) {
    return;
  }
#if MBUS_RTU_PLATFORM_TRACE
  MbusPlatform::trace(MbusPlatformTraceRecord(
      MbusPlatformTraceEvent::EventTaskStarted,
      MbusPlatform::microsNow()));
#endif
#if MBUS_RTU_RX_THREAD_STACK_WATERMARK_ACTIVE
  _sampleRxThreadStack();
#endif
  while (MbusPlatform::eventTaskRunning(_rxThreadState)) {
#if MBUS_RTU_PLATFORM_TRACE
    MbusPlatform::trace(MbusPlatformTraceRecord(
        MbusPlatformTraceEvent::EventTaskWaiting,
        MbusPlatform::microsNow()));
#endif
    MbusPlatform::waitEvent(_rxThreadState);
    if (!MbusPlatform::eventTaskRunning(_rxThreadState)) {
      break;
    }
#if MBUS_RTU_PLATFORM_TRACE
    MbusPlatform::trace(MbusPlatformTraceRecord(
        MbusPlatformTraceEvent::EventTaskWoken,
        MbusPlatform::microsNow()));
#endif
    _rxReadableHint.store(false, MBUS_MEM_RELAXED);
    _ingestPoll();
#if MBUS_RTU_RX_THREAD_STACK_WATERMARK_ACTIVE
    _sampleRxThreadStack();
#endif
  }
#if MBUS_RTU_PLATFORM_TRACE
  MbusPlatform::trace(MbusPlatformTraceRecord(
      MbusPlatformTraceEvent::EventTaskStopped,
      MbusPlatform::microsNow()));
#endif
#endif
}

#if MBUS_RTU_RX_THREAD_STACK_WATERMARK_ACTIVE
void ModbusRTUComm::_sampleRxThreadStack() {
  const modbus_rtu::platform::TaskStackSnapshot snapshot =
      MbusPlatform::currentTaskStack();
  if (!snapshot.valid) {
    return;
  }
  const uint32_t sizeBytes = snapshot.size_bytes;
  const uint32_t freeBytes = snapshot.free_bytes;

  if (sizeBytes > 0U) {
    _rxThreadStackSizeBytes.store(sizeBytes, MBUS_MEM_RELAXED);
  }
  _rxThreadStackLastFreeBytes.store(freeBytes, MBUS_MEM_RELAXED);

  const uint32_t prevMin = _rxThreadStackMinFreeBytes.load(MBUS_MEM_RELAXED);
  if (prevMin == 0U || freeBytes < prevMin) {
    _rxThreadStackMinFreeBytes.store(freeBytes, MBUS_MEM_RELAXED);
  }
  _rxThreadStackSamples.fetch_add(1U, MBUS_MEM_RELAXED);
}

ModbusRTUComm::RxThreadStackSnapshot
ModbusRTUComm::rxThreadStackSnapshot() const {
  RxThreadStackSnapshot out{};
  out.sizeBytes = _rxThreadStackSizeBytes.load(MBUS_MEM_RELAXED);
  out.freeNowBytes = _rxThreadStackLastFreeBytes.load(MBUS_MEM_RELAXED);
  out.minFreeBytes = _rxThreadStackMinFreeBytes.load(MBUS_MEM_RELAXED);
  out.maxUsedBytes =
      (out.sizeBytes >= out.minFreeBytes) ? (out.sizeBytes - out.minFreeBytes) : 0U;
  out.samples = _rxThreadStackSamples.load(MBUS_MEM_RELAXED);
  return out;
}
#endif

void ModbusRTUComm::_ingestPoll() {
  while (MbusPlatform::available(_serial) > 0) {
    const int next = MbusPlatform::read(_serial);
    if (next < 0) {
      break;
    }
    const uint32_t timestampUs = MbusPlatform::microsNow();
#if MBUS_RTU_PLATFORM_TRACE
    MbusPlatform::trace(MbusPlatformTraceRecord(
        MbusPlatformTraceEvent::IngressByte,
        timestampUs,
        static_cast<uint8_t>(next)));
#endif
    _pushRxByte(static_cast<uint8_t>(next), timestampUs);
  }
}

bool ModbusRTUComm::_pushRxByte(uint8_t byte, uint32_t tsUs) {
  const uint32_t cap = static_cast<uint32_t>(MBUS_RTU_RX_RING_SIZE > 0 ? MBUS_RTU_RX_RING_SIZE : 1);
  const uint32_t head = _rxHead.load(MBUS_MEM_RELAXED);
  uint32_t next = head + 1U;
  if (next >= cap) {
    next = 0;
  }
  const uint32_t tail = _rxTail.load(MBUS_MEM_ACQUIRE);
  if (next == tail) {
    const uint32_t overflows = _rxOverflowCount.fetch_add(1U, MBUS_MEM_RELAXED) + 1U;
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
    _dbg.rx_overflow_count = overflows;
#else
    (void)overflows;
#endif
    return false;
  }

  _rxRing[head].byte = byte;
  _rxRing[head].ts_us = tsUs;
  _lastRxByteUs.store(tsUs, MBUS_MEM_RELAXED);
  _lastTrafficMs.store(MbusPlatform::millisNow(), MBUS_MEM_RELAXED);
  _rxHead.store(next, MBUS_MEM_RELEASE);
  return true;
}

bool ModbusRTUComm::_popRxByte(uint8_t& byteOut, uint32_t& tsUsOut) {
  const uint32_t cap = static_cast<uint32_t>(MBUS_RTU_RX_RING_SIZE > 0 ? MBUS_RTU_RX_RING_SIZE : 1);
  const uint32_t tail = _rxTail.load(MBUS_MEM_RELAXED);
  const uint32_t head = _rxHead.load(MBUS_MEM_ACQUIRE);
  if (head == tail) {
    return false;
  }
  const RxEntry& entry = _rxRing[tail];
  byteOut = entry.byte;
  tsUsOut = entry.ts_us;
  uint32_t next = tail + 1U;
  if (next >= cap) {
    next = 0;
  }
  _rxTail.store(next, MBUS_MEM_RELEASE);
  return true;
}

bool ModbusRTUComm::_peekRx(uint16_t offset, RxEntry& out) const {
  const uint32_t cap = static_cast<uint32_t>(MBUS_RTU_RX_RING_SIZE > 0 ? MBUS_RTU_RX_RING_SIZE : 1);
  const uint32_t tail = _rxTail.load(MBUS_MEM_RELAXED);
  const uint32_t head = _rxHead.load(MBUS_MEM_ACQUIRE);
  const uint32_t count = (head >= tail) ? (head - tail) : (cap - (tail - head));
  if (offset >= count) {
    return false;
  }
  uint32_t idx = tail + static_cast<uint32_t>(offset);
  if (idx >= cap) {
    idx -= cap;
  }
  out = _rxRing[idx];
  return true;
}

uint16_t ModbusRTUComm::_rxCount() const {
  const uint32_t cap = static_cast<uint32_t>(MBUS_RTU_RX_RING_SIZE > 0 ? MBUS_RTU_RX_RING_SIZE : 1);
  const uint32_t tail = _rxTail.load(MBUS_MEM_RELAXED);
  const uint32_t head = _rxHead.load(MBUS_MEM_ACQUIRE);
  const uint32_t count = (head >= tail) ? (head - tail) : (cap - (tail - head));
  return static_cast<uint16_t>(count);
}

bool ModbusRTUComm::_rxEmpty() const {
  const uint32_t tail = _rxTail.load(MBUS_MEM_RELAXED);
  const uint32_t head = _rxHead.load(MBUS_MEM_ACQUIRE);
  return head == tail;
}

void ModbusRTUComm::_dropRxBytes(uint16_t n) {
  uint8_t b = 0;
  uint32_t ts = 0;
  for (uint16_t i = 0; i < n; ++i) {
    if (!_popRxByte(b, ts)) {
      break;
    }
  }
}

ModbusRTUComm::ParseLenStatus ModbusRTUComm::_expectedFrameLength(
    const uint8_t* buf,
    uint16_t have,
    uint16_t& expectedLen) const {
  expectedLen = 0;
  if (!buf || have < 2) {
    return ParseLenStatus::NeedMore;
  }

  const uint8_t fc = buf[1];

  if ((fc & 0x80U) != 0U) {
    expectedLen = 5;
    return ParseLenStatus::HaveLen;
  }

  switch (fc) {
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x04:
      if (have < 3) {
        return ParseLenStatus::NeedMore;
      }
      expectedLen = static_cast<uint16_t>(5U + buf[2]);
      if (expectedLen > 256U) {
        return ParseLenStatus::Impossible;
      }
      return ParseLenStatus::HaveLen;

    case 0x05:
    case 0x06:
    case 0x0F:
    case 0x10:
      expectedLen = 8;
      return ParseLenStatus::HaveLen;

    default:
      return ParseLenStatus::NeedMore;
  }
}

uint16_t ModbusRTUComm::_crc16(const uint8_t* buf, uint16_t lenWithoutCrc) const {
  uint16_t value = 0xFFFF;
  for (uint16_t i = 0; i < lenWithoutCrc; ++i) {
    value ^= static_cast<uint16_t>(buf[i]);
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

ModbusRTUComm::ExtractStatus ModbusRTUComm::_extractNextFrame(FrameView& frame) {
  frame = FrameView{};

  const uint16_t available = _rxCount();
  if (available == 0) {
    return ExtractStatus::None;
  }

  // A one-to-three-byte fragment can never be a complete RTU frame. Once the
  // line has been silent for T3.5, consume it as a framing error instead of
  // forcing _drainToIdle() to wait for its hard 250 ms escape.
  if (available < 4) {
    RxEntry last{};
    if (!_peekRx(static_cast<uint16_t>(available - 1U), last)) {
      return ExtractStatus::None;
    }
    if (static_cast<uint32_t>(MbusPlatform::microsNow() - last.ts_us) < _frameTimeout) {
      return ExtractStatus::None;
    }
    _dropRxBytes(available);
    return ExtractStatus::GapViolation;
  }

  const uint16_t scanCap = min_val<uint16_t>(available, 256U);
  uint8_t* bytes = _extractScratchBytes;
  uint32_t* ts = _extractScratchTs;
  for (uint16_t i = 0; i < scanCap; ++i) {
    RxEntry entry{};
    if (!_peekRx(i, entry)) {
      return ExtractStatus::None;
    }
    bytes[i] = entry.byte;
    ts[i] = entry.ts_us;
  }

  uint16_t expectedLen = 0;
  bool expectedKnown = false;
  bool completeByLen = false;
  bool boundaryFound = false;
  uint16_t boundaryPos = 0;
  bool t15Violation = false;

  uint16_t candidateLen = 0;
  for (uint16_t i = 0; i < scanCap; ++i) {
    if (i > 0) {
      const uint32_t gap = ts[i] - ts[i - 1];
      if (gap > _charTimeout) {
        // Spec handling: any >T1.5 inter-char gap marks frame NOK.
        t15Violation = true;
        if (gap >= _frameTimeout) {
          // End-of-frame boundary reached; everything before this index is one frame candidate.
          boundaryFound = true;
          boundaryPos = i;
          break;
        }
      }
    }

    candidateLen = static_cast<uint16_t>(i + 1);
    uint16_t localExpected = 0;
    const ParseLenStatus parse = _expectedFrameLength(bytes, candidateLen, localExpected);
    if (parse == ParseLenStatus::Impossible) {
      _dropRxBytes(1);
      return ExtractStatus::FrameError;
    }
    if (parse == ParseLenStatus::HaveLen) {
      expectedKnown = true;
      expectedLen = localExpected;
      if (expectedLen < 4 || expectedLen > 256) {
        _dropRxBytes(1);
        return ExtractStatus::FrameError;
      }
      if (candidateLen > expectedLen) {
        _dropRxBytes(1);
        return ExtractStatus::FrameError;
      }
      // Only accept length-complete frames when no T1.5 violation occurred.
      if (candidateLen == expectedLen && !t15Violation) {
        completeByLen = true;
        break;
      }
    }
  }

  if (!completeByLen) {
    if (boundaryFound) {
      candidateLen = boundaryPos;
      if (candidateLen == 0) {
        _dropRxBytes(1);
        return ExtractStatus::FrameError;
      }
      if (expectedKnown && candidateLen < expectedLen) {
        _dropRxBytes(candidateLen);
        return ExtractStatus::GapViolation;
      }
    } else {
      if (candidateLen == 0) {
        return ExtractStatus::None;
      }
      const uint32_t nowUs = MbusPlatform::microsNow();
      const uint32_t sinceLast = nowUs - ts[candidateLen - 1];
      // Without an explicit boundary, only finalize once idle >= T3.5.
      if (sinceLast < _frameTimeout) {
        return ExtractStatus::None;
      }
      if (expectedKnown && candidateLen < expectedLen) {
        _dropRxBytes(candidateLen);
        return ExtractStatus::GapViolation;
      }
    }
  }

  if (candidateLen < 4 || candidateLen > 256) {
    _dropRxBytes(max_val<uint16_t>(candidateLen, 1));
    return ExtractStatus::FrameError;
  }

  if (t15Violation) {
    _dropRxBytes(candidateLen);
    return ExtractStatus::GapViolation;
  }

  const uint16_t calc = _crc16(bytes, static_cast<uint16_t>(candidateLen - 2));
  const uint16_t got = static_cast<uint16_t>(bytes[candidateLen - 2]) |
                       static_cast<uint16_t>(bytes[candidateLen - 1] << 8);
  if (calc != got) {
    _dropRxBytes(1);
    return ExtractStatus::CrcError;
  }

  for (uint16_t i = 0; i < candidateLen; ++i) {
    frame.data[i] = bytes[i];
  }
  frame.len = candidateLen;
  frame.first_us = ts[0];
  frame.last_us = ts[candidateLen - 1];
  frame.t15_violation = false;

  _dropRxBytes(candidateLen);
  return ExtractStatus::Ok;
}

bool ModbusRTUComm::_frameMatchesRequest(const FrameView& frame, uint8_t expectedUnit, uint8_t expectedFc) const {
  if (frame.len < 2) {
    return false;
  }
  if (frame.data[0] != expectedUnit) {
    return false;
  }
  const uint8_t fc = frame.data[1];
  if (fc == expectedFc) {
    return true;
  }
  return fc == static_cast<uint8_t>(expectedFc | 0x80U);
}

bool ModbusRTUComm::_isRecentDuplicate(const FrameView& frame) const {
  if (_lastAcceptedLen == 0 || frame.len != _lastAcceptedLen) {
    return false;
  }
  if (_lastAcceptedTxSeq == _txSeq) {
    return false;
  }
  if ((frame.last_us - _lastAcceptedUs) > MBUS_RTU_DEDUPE_WINDOW_US) {
    return false;
  }
  return memcmp(frame.data, _lastAcceptedFrame, frame.len) == 0;
}

void ModbusRTUComm::_rememberAcceptedFrame(const FrameView& frame) {
  _lastAcceptedLen = frame.len;
  _lastAcceptedUs = frame.last_us;
  _lastAcceptedTxSeq = _txSeq;
  if (frame.len > 0) {
    memcpy(_lastAcceptedFrame, frame.data, frame.len);
  }
}

void ModbusRTUComm::_armTimedOutPending(ModbusADU& reqAdu, uint32_t nowUs, uint32_t graceUs) {
  _timedOutPending = TimedOutPending{};

  const uint8_t unit = reqAdu.getUnitId();
  const uint8_t fc = reqAdu.getFunctionCode();
  if (unit == 0 || fc == 0x45) {
    return;
  }

  _timedOutPending.valid = true;
  _timedOutPending.unit = unit;
  _timedOutPending.fc = static_cast<uint8_t>(fc & 0x7FU);
  _timedOutPending.word0 = reqAdu.getDataRegister(0);
  _timedOutPending.word2 = reqAdu.getDataRegister(2);

  if (fc == 0x01 || fc == 0x02) {
    _timedOutPending.byte_count = static_cast<uint8_t>(div8RndUp(reqAdu.getDataRegister(2)));
  } else if (fc == 0x03 || fc == 0x04) {
    _timedOutPending.byte_count = static_cast<uint8_t>(reqAdu.getDataRegister(2) * 2U);
  } else {
    _timedOutPending.byte_count = 0;
  }

  _timedOutPending.grace_until_us = nowUs + graceUs;
}

void ModbusRTUComm::_expireTimedOutPending(uint32_t nowUs) {
  if (!_timedOutPending.valid) {
    return;
  }
  if (static_cast<int32_t>(nowUs - _timedOutPending.grace_until_us) >= 0) {
    _timedOutPending.valid = false;
  }
}

bool ModbusRTUComm::_matchAndConsumeTimedOutPending(const FrameView& frame, uint32_t nowUs) {
  _expireTimedOutPending(nowUs);
  if (!_timedOutPending.valid) {
    return false;
  }
  if (frame.len < 2) {
    return false;
  }
  if (frame.data[0] != _timedOutPending.unit) {
    return false;
  }

  const uint8_t rxFc = frame.data[1];
  const uint8_t rxBaseFc = static_cast<uint8_t>(rxFc & 0x7FU);
  if (rxBaseFc != _timedOutPending.fc) {
    return false;
  }

  bool match = false;
  if ((rxFc & 0x80U) != 0U) {
    match = true;
  } else {
    switch (rxBaseFc) {
      case 0x01:
      case 0x02:
      case 0x03:
      case 0x04:
        match = (frame.len >= 5U) && (frame.data[2] == _timedOutPending.byte_count);
        break;
      case 0x05:
      case 0x06:
      case 0x0F:
      case 0x10:
        if (frame.len >= 8U) {
          const uint16_t word0 = static_cast<uint16_t>(frame.data[2] << 8) | frame.data[3];
          const uint16_t word2 = static_cast<uint16_t>(frame.data[4] << 8) | frame.data[5];
          match = (word0 == _timedOutPending.word0) && (word2 == _timedOutPending.word2);
        }
        break;
      default:
        match = true;
        break;
    }
  }

  if (match) {
    _timedOutPending.valid = false;
  }
  return match;
}

bool ModbusRTUComm::_drainToIdle() {
  // Evaluate the existing stale-gate watchdog before polling the serial
  // backend. A byte retained there during a long idle period receives a fresh
  // timestamp when ingested; clearing first prevents that fresh RX evidence
  // from reviving an old gate at either the signed half-range or a full
  // micros() wrap. This branch is inert throughout normal millisecond-scale
  // RTU scheduling.
  const uint32_t preDrainNowMs = MbusPlatform::millisNow();
  _expireTxGateByCoarseAge(preDrainNowMs);
  const uint32_t preDrainLastTrafficMs =
      _lastTrafficMs.load(MBUS_MEM_RELAXED);
  if(preDrainLastTrafficMs != 0U &&
     static_cast<uint32_t>(preDrainNowMs - preDrainLastTrafficMs) >
     _idleResetMs){
    _replaceTxGate(MbusPlatform::microsNow(), 0U);
  }

  _expireTimedOutPending(MbusPlatform::microsNow());
  const uint32_t drainStartUs = MbusPlatform::microsNow();

  while (true) {
    const uint32_t loopStartUs = MbusPlatform::microsNow();
    if (!_rxThreadRunning) {
      _ingestPoll();
    }

    FrameView ignored{};
    while (true) {
      const ExtractStatus st = _extractNextFrame(ignored);
      if (st == ExtractStatus::None) {
        break;
      }
      const bool matchedTimedOutPending =
          (st == ExtractStatus::Ok) && _matchAndConsumeTimedOutPending(ignored, ignored.last_us);
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
      if (matchedTimedOutPending) {
        _dbg.recovered_late_count++;
      } else {
        _dbg.recovered_drain_drop_count++;
      }
#else
      (void)matchedTimedOutPending;
#endif
      (void)st;
    }

    const uint32_t nowUs = MbusPlatform::microsNow();
    _expireTimedOutPending(nowUs);
    const uint32_t lastRxUs = _lastRxByteUs.load(MBUS_MEM_ACQUIRE);
    const uint32_t sinceRx = (lastRxUs == 0) ? nowUs : (nowUs - lastRxUs);
    if (_rxEmpty() && sinceRx >= _frameTimeout) {
      // RX-idle proof may extend the next-TX boundary, but it must never erase
      // a live no-response, broadcast, or one-shot holdoff armed by writeAdu().
      // Comparing bounded remaining durations also means a dormant, expired
      // gate cannot be revived by freshly ingesting a byte after long idle.
      _extendTxGate(nowUs, _baseExtraGapUs);
      return true;
    }

#if MBUS_RTU_DRAIN_ESCAPE_US > 0
    const uint32_t drainElapsedUs = nowUs - drainStartUs;
    if (drainElapsedUs >= MBUS_RTU_DRAIN_ESCAPE_US) {
#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED || \
    (defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS)
      const uint16_t queued = _rxCount();
#endif
#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED
      const uint32_t sinceLastRxUs = (lastRxUs == 0) ? 0 : (nowUs - lastRxUs);
#endif

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
      _dbg.drain_escape_count++;
      _dbg.last_drain_escape_us = drainElapsedUs;
      _dbg.last_drain_escape_rx_count = queued;
#endif

#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED
      if (mbusDiagSerialTryLock()) {
        if (mbusDiagSerialCanWrite(120U)) {
          Serial.print("[MBUS_RTU] hard-fail drain escape us=");
          Serial.print(static_cast<unsigned long>(drainElapsedUs));
          Serial.print(" queued=");
          Serial.print(static_cast<unsigned>(queued));
          Serial.print(" since_rx_us=");
          Serial.print(static_cast<unsigned long>(sinceLastRxUs));
          Serial.print(" frame_to_us=");
          Serial.println(static_cast<unsigned long>(_frameTimeout));
        }
        mbusDiagSerialUnlock();
      }
#endif

      // Hard recovery: clear pending RX state and force a known-safe TX gate.
      const uint32_t head = _rxHead.load(MBUS_MEM_ACQUIRE);
      _rxTail.store(head, MBUS_MEM_RELEASE);
      _lastRxByteUs.store(0, MBUS_MEM_RELAXED);
      _timedOutPending.valid = false;
      const uint32_t recoveryGateStartUs = MbusPlatform::microsNow();
      _replaceTxGate(
          recoveryGateStartUs,
          static_cast<uint64_t>(_frameTimeout) + _baseExtraGapUs);
      _lastTrafficMs.store(MbusPlatform::millisNow(), MBUS_MEM_RELAXED);
      return false;
    }
#endif

    _yield_if_long_gap(loopStartUs);
  }
}

uint32_t ModbusRTUComm::_computeLateGraceUs() const {
  const uint32_t dyn = _charTimeUs * 4UL;
  return clamp_u32(dyn, MBUS_RTU_LATE_GRACE_US_MIN, MBUS_RTU_LATE_GRACE_US_MAX);
}

// readAdu transaction contract:
// - Parse RTU responses from a continuous byte stream (possibly multi-frame).
// - Preserve operator-visible terminal errors (timeout/frame/crc).
// - Keep recovered anomalies internal (late/duplicate/stray/t15).
// - Always drain to RTU-idle before returning (success or failure).
ModbusRTUCommError ModbusRTUComm::readAdu(ModbusADU& adu) {
  const uint8_t expectedUnit = adu.getUnitId();
  const uint8_t expectedFc = adu.getFunctionCode();
  // Always reset ADU output upfront. Only PROCESS_FRAME matched-path repopulates.
  adu.setRtuLen(0);

  RxTxnCtx ctx{};
  ctx.start_us = MbusPlatform::microsNow();
  ctx.wait_start_us = ctx.start_us;

  const uint32_t lateGraceUs = _computeLateGraceUs();
  const uint64_t requestedReadTimeoutUsWide =
      static_cast<uint64_t>(_readTimeout) * 1000ULL;
  const uint32_t requestedReadTimeoutUs =
      requestedReadTimeoutUsWide > 0xFFFFFFFFULL
          ? 0xFFFFFFFFUL
          : static_cast<uint32_t>(requestedReadTimeoutUsWide);
  const RxTimingBudget timingBudget = make_rx_timing_budget(
      requestedReadTimeoutUs, lateGraceUs, _maxFrameReceiveUs);
  ctx.read_timeout_us = timingBudget.read_timeout_us;
  ctx.first_byte_deadline_us = ctx.start_us + ctx.read_timeout_us;

  // Last-resort transaction escape, anchored only to readAdu() entry. Normal
  // termination remains governed by first-byte, RTU frame, recovery, and late
  // deadlines. The mandatory DRAIN phase has its own MBUS_RTU_DRAIN_ESCAPE_US
  // bound, so the total default return bound is active_escape_us plus drain.
  const uint32_t activeEscapeUs = timingBudget.active_escape_us;

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
  _dbg.last_read_start_us = ctx.start_us;
  _dbg.last_rx_len = 0;
  _dbg.last_rx_first_us = 0;
  _dbg.last_rx_last_us = 0;
  _dbg.last_read_total_us = 0;
  _dbg.last_wait_first_us = 0;
  _dbg.last_err = MODBUS_RTU_COMM_SUCCESS;
  _dbg.state_last = static_cast<uint8_t>(ctx.state);
  _dbg.state_current = static_cast<uint8_t>(ctx.state);
#endif

#if MBUS_RTU_PLATFORM_TRACE
  uint32_t stateTraceUs = ctx.start_us;
#endif
  // Centralized transition helper so all state changes share identical
  // instrumentation/diagnostics behavior.
  auto transition = [&](RxTxnState next) {
    if (ctx.state == next) {
      return;
    }
    const RxTxnState prev = ctx.state;
    ctx.state = next;
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
    _dbg.state_transition_count++;
    _dbg.state_last = static_cast<uint8_t>(prev);
    _dbg.state_current = static_cast<uint8_t>(next);
#endif
#if MBUS_RTU_TRACE_STATES
    if (mbusDiagSerialTryLock()) {
      if (mbusDiagSerialCanWrite(48U)) {
        Serial.print("[MBUS_RTU_STATE] ");
        Serial.print(static_cast<unsigned>(static_cast<uint8_t>(prev)));
        Serial.print("->");
        Serial.println(static_cast<unsigned>(static_cast<uint8_t>(next)));
      }
      mbusDiagSerialUnlock();
    }
#endif
#if MBUS_RTU_PLATFORM_TRACE
    MbusPlatform::trace(MbusPlatformTraceRecord(
        MbusPlatformTraceEvent::RxStateTransition,
        stateTraceUs,
        (static_cast<uint32_t>(static_cast<uint8_t>(prev)) << 8U) |
            static_cast<uint8_t>(next)));
#endif
#if !(defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS) && !MBUS_RTU_TRACE_STATES
    (void)prev;
#endif
  };

  auto noteFirstRxActivity = [&](uint32_t eventUs, uint32_t observedNowUs) {
    if (ctx.saw_rx_activity) {
      return;
    }
    const uint32_t normalizedEventUs =
        normalize_rx_event_timestamp(
            ctx.wait_start_us, observedNowUs, eventUs, activeEscapeUs);
    ctx.saw_rx_activity = true;
    ctx.first_byte_us = normalizedEventUs;
    ctx.recovery_deadline_us = normalizedEventUs + ctx.read_timeout_us;
    ctx.frame_envelope_deadline_us = normalizedEventUs + _maxFrameReceiveUs;
  };

  auto activeEscapeExpired = [&](uint32_t nowUs) {
    return rx_interval_elapsed(ctx.start_us, nowUs, activeEscapeUs);
  };

  auto enterDrainOnActiveEscape = [&](uint32_t nowUs) {
    if (!activeEscapeExpired(nowUs)) {
      return false;
    }

#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED || \
    (defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS)
    const uint16_t queued = _rxCount();
    const uint32_t elapsedUs = static_cast<uint32_t>(nowUs - ctx.start_us);
#endif

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
    _dbg.transaction_escape_count++;
    _dbg.last_transaction_escape_us = elapsedUs;
    _dbg.last_transaction_escape_budget_us = activeEscapeUs;
    _dbg.last_transaction_escape_rx_count = queued;
    _dbg.last_transaction_escape_state = static_cast<uint8_t>(ctx.state);
#endif

#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED
    if (mbusDiagSerialTryLock()) {
      if (mbusDiagSerialCanWrite(128U)) {
        Serial.print("[MBUS_RTU] transaction escape us=");
        Serial.print(static_cast<unsigned long>(elapsedUs));
        Serial.print(" budget_us=");
        Serial.print(static_cast<unsigned long>(activeEscapeUs));
        Serial.print(" state=");
        Serial.print(static_cast<unsigned>(static_cast<uint8_t>(ctx.state)));
        Serial.print(" queued=");
        Serial.println(static_cast<unsigned>(queued));
      }
      mbusDiagSerialUnlock();
    }
#endif

    transition(RxTxnState::DRAIN);
    return true;
  };

  auto resumeAfterProcessedCandidate = [&](uint32_t nowUs) {
    // PROCESS_FRAME intentionally classifies one already-extracted candidate
    // before checking H so a matching response at the legal boundary wins.
    // An abnormal non-match can overshoot only one classifier step (and, once
    // per burst, one cooperative yield) before this common escape check.
    if (!enterDrainOnActiveEscape(nowUs)) {
      transition(ctx.resume_after_process);
    }
  };

  // Canonical transaction loop:
  // WAIT_FIRST -> RX_ACTIVE/LATE_WINDOW -> PROCESS_FRAME ... -> DRAIN -> DONE
  while (ctx.state != RxTxnState::DONE) {
    if (!_rxThreadRunning) {
      // Poll ingress remains canonical; callback/rx-thread mode is additive.
      _ingestPoll();
    }

    const uint32_t nowUs = MbusPlatform::microsNow();
#if MBUS_RTU_PLATFORM_TRACE
    stateTraceUs = nowUs;
#endif
    _expireTimedOutPending(nowUs);

    // A retained timestamp is not proof that this transaction received data:
    // after the signed micros() half-range it can appear to be in the future.
    // The ring head is release-published by the producer, so pending bytes are
    // the authoritative current-transaction activity signal.
    if (!ctx.saw_rx_activity && !_rxEmpty()) {
      RxEntry first{};
      const uint32_t eventUs = _peekRx(0, first) ? first.ts_us : nowUs;
      noteFirstRxActivity(eventUs, nowUs);
      if (ctx.state == RxTxnState::WAIT_FIRST ||
          ctx.state == RxTxnState::LATE_WINDOW) {
        transition(RxTxnState::RX_ACTIVE);
      }
    }

    switch (ctx.state) {
      case RxTxnState::WAIT_FIRST: {
        // Split-timeout phase 1: wait for first byte/frame, then upgrade to
        // RX_ACTIVE. A first-byte timeout arms late-window logic instead of
        // immediately returning timeout, preventing retry collisions.
        ctx.pending_extract = _extractNextFrame(ctx.pending_frame);
        if (ctx.pending_extract != ExtractStatus::None) {
          const uint32_t eventUs = (ctx.pending_frame.first_us != 0) ? ctx.pending_frame.first_us : nowUs;
          if (!ctx.saw_rx_activity) {
            noteFirstRxActivity(eventUs, nowUs);
          }
          ctx.resume_after_process = RxTxnState::RX_ACTIVE;
          transition(RxTxnState::PROCESS_FRAME);
          continue;
        }

        if (enterDrainOnActiveEscape(nowUs)) {
          continue;
        }

        if (!ctx.saw_rx_activity && static_cast<int32_t>(nowUs - ctx.first_byte_deadline_us) >= 0) {
          _armTimedOutPending(adu, nowUs, lateGraceUs);
          ctx.late_window = true;
          ctx.late_deadline_us = nowUs + lateGraceUs;
          MBUS_RTU_ASSERT(_timedOutPending.valid || expectedUnit == 0 || expectedFc == 0x45);
          transition(RxTxnState::LATE_WINDOW);
          continue;
        }

        ctx.burst_budget = MBUS_RTU_PROCESS_BURST_CAP;
        _yield_if_long_gap(ctx.wait_start_us);
        break;
      }

      case RxTxnState::RX_ACTIVE: {
        // Split-timeout phase 2: bytes are flowing, so timeout now means in-frame
        // deadline expiry. We still process every extracted event through
        // PROCESS_FRAME for classification.
        ctx.pending_extract = _extractNextFrame(ctx.pending_frame);
        if (ctx.pending_extract != ExtractStatus::None) {
          ctx.resume_after_process = RxTxnState::RX_ACTIVE;
          transition(RxTxnState::PROCESS_FRAME);
          continue;
        }

        if (enterDrainOnActiveEscape(nowUs)) {
          continue;
        }

        if (ctx.saw_rx_activity &&
            static_cast<int32_t>(nowUs - ctx.frame_envelope_deadline_us) >= 0) {
          transition(RxTxnState::DRAIN);
          continue;
        }

        // Once a complete/invalid candidate has been consumed and the line is
        // RTU-idle, retain the historical recovery window for a following
        // clean frame. A still-active partial frame is governed solely by
        // T1.5/T3.5 and the defensive maximum-frame envelope above.
        if (_rxEmpty()) {
          const uint32_t lastRxUs = _lastRxByteUs.load(MBUS_MEM_ACQUIRE);
          const bool lineIdle = lastRxUs != 0U &&
              static_cast<uint32_t>(nowUs - lastRxUs) >= _frameTimeout;
          if (lineIdle &&
              static_cast<int32_t>(nowUs - ctx.recovery_deadline_us) >= 0) {
            transition(RxTxnState::DRAIN);
            continue;
          }
        }

        ctx.burst_budget = MBUS_RTU_PROCESS_BURST_CAP;
        _yield_if_long_gap(ctx.wait_start_us);
        break;
      }

      case RxTxnState::LATE_WINDOW: {
        // Late-window accepts delayed valid response safely while TX is blocked.
        // When grace expires, we terminate via DRAIN and surface terminal status.
        MBUS_RTU_ASSERT(ctx.late_window);
        ctx.pending_extract = _extractNextFrame(ctx.pending_frame);
        if (ctx.pending_extract != ExtractStatus::None) {
          ctx.resume_after_process = RxTxnState::LATE_WINDOW;
          transition(RxTxnState::PROCESS_FRAME);
          continue;
        }

        if (enterDrainOnActiveEscape(nowUs)) {
          continue;
        }

        if (static_cast<int32_t>(nowUs - ctx.late_deadline_us) >= 0) {
          transition(RxTxnState::DRAIN);
          continue;
        }

        ctx.burst_budget = MBUS_RTU_PROCESS_BURST_CAP;
        _yield_if_long_gap(ctx.wait_start_us);
        break;
      }

      case RxTxnState::PROCESS_FRAME: {
        // Single-frame classifier. Every extractor outcome is normalized here so
        // terminal decision logic is explicit and testable.
        const ExtractStatus extracted = ctx.pending_extract;
        ctx.pending_extract = ExtractStatus::None;

        if (extracted == ExtractStatus::None) {
          ctx.burst_budget = MBUS_RTU_PROCESS_BURST_CAP;
          resumeAfterProcessedCandidate(nowUs);
          continue;
        }

        if (ctx.burst_budget > 0) {
          --ctx.burst_budget;
        }
        if (ctx.burst_budget == 0) {
          // Throughput/fairness tradeoff: prevent parser monopolization when a
          // burst contains many frames or repeated garbage.
          MbusPlatform::yieldTask();
          ctx.burst_budget = MBUS_RTU_PROCESS_BURST_CAP;
        }

        if (extracted == ExtractStatus::CrcError) {
          ctx.saw_crc_err = true;
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
          _dbg.recovered_parse_count++;
#endif
          resumeAfterProcessedCandidate(nowUs);
          continue;
        }

        if (extracted == ExtractStatus::GapViolation) {
          ctx.saw_frame_err = true;
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
          _dbg.recovered_t15_count++;
#endif
          resumeAfterProcessedCandidate(nowUs);
          continue;
        }

        if (extracted == ExtractStatus::FrameError) {
          ctx.saw_frame_err = true;
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
          _dbg.recovered_parse_count++;
#endif
          resumeAfterProcessedCandidate(nowUs);
          continue;
        }

        if (extracted != ExtractStatus::Ok) {
          resumeAfterProcessedCandidate(nowUs);
          continue;
        }

        if (!ctx.saw_rx_activity) {
          const uint32_t eventUs = (ctx.pending_frame.first_us != 0) ? ctx.pending_frame.first_us : nowUs;
          noteFirstRxActivity(eventUs, nowUs);
        }

        ctx.last_byte_us = ctx.pending_frame.last_us;

        if (_isRecentDuplicate(ctx.pending_frame)) {
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
          _dbg.recovered_duplicate_count++;
#endif
          resumeAfterProcessedCandidate(nowUs);
          continue;
        }

        if (_frameMatchesRequest(ctx.pending_frame, expectedUnit, expectedFc)) {
          // Only this path commits ADU output and marks terminal success.
          if (ctx.late_window) {
            ctx.late_accepted = true;
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
            _dbg.recovered_late_count++;
            _dbg.late_match_after_timeout_count++;
#endif
          }

          for (uint16_t i = 0; i < ctx.pending_frame.len; ++i) {
            adu.rtu[i] = ctx.pending_frame.data[i];
          }
          adu.setRtuLen(ctx.pending_frame.len);
          _rememberAcceptedFrame(ctx.pending_frame);
          _timedOutPending.valid = false;
          ctx.saw_match = true;
          transition(RxTxnState::DRAIN);
          continue;
        }

        if (_matchAndConsumeTimedOutPending(ctx.pending_frame, ctx.pending_frame.last_us)) {
          // Safe recovery path: frame belongs to previously timed-out request.
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
          _dbg.recovered_late_count++;
          _dbg.late_match_after_timeout_count++;
#endif
          resumeAfterProcessedCandidate(nowUs);
          continue;
        }

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
        // Valid but unrelated frame while request is active; treated as stray.
        _dbg.recovered_stray_count++;
#endif
        resumeAfterProcessedCandidate(nowUs);
        continue;
      }

      case RxTxnState::DRAIN:
        // Hard barrier: never exit read path until line is drained to RTU-idle.
        if (!_drainToIdle()) {
          ctx.saw_frame_err = true;
          ctx.saw_match = false;
        }
        ctx.did_drain = true;
        transition(RxTxnState::DONE);
        break;

      case RxTxnState::DONE:
      default:
        break;
    }
  }

  MBUS_RTU_ASSERT(ctx.did_drain);

  if (ctx.saw_match) {
    // Success path: keep operator-visible behavior unchanged.
    _lastTrafficMs.store(MbusPlatform::millisNow(), MBUS_MEM_RELAXED);
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
    _dbg.last_wait_first_us = ctx.first_byte_us
                                  ? elapsed_us_or_zero(ctx.wait_start_us, ctx.first_byte_us)
                                  : elapsed_us_or_zero(ctx.wait_start_us, MbusPlatform::microsNow());
    _dbg.last_rx_first_us = ctx.first_byte_us;
    _dbg.last_rx_last_us = ctx.last_byte_us;
    _dbg.last_read_total_us = ctx.last_byte_us
                                  ? elapsed_us_or_zero(ctx.wait_start_us, ctx.last_byte_us)
                                  : 0U;
    _dbg.last_rx_len = adu.getRtuLen();
    _dbg.last_err = MODBUS_RTU_COMM_SUCCESS;
    _dbg.ok_count++;
#endif
#ifdef MBUS_PRINTALL
    _dumpFrame("RX ◀", adu.rtu, adu.getRtuLen());
#endif
    (void)ctx.late_accepted;
    return MODBUS_RTU_COMM_SUCCESS;
  }

  adu.setRtuLen(0);

  // Preserve historical terminal error precedence:
  // frame-error > crc-error > timeout.
  ModbusRTUCommError outErr = MODBUS_RTU_COMM_TIMEOUT;
  if (ctx.saw_frame_err) {
    outErr = MODBUS_RTU_COMM_FRAME_ERROR;
  } else if (ctx.saw_crc_err) {
    outErr = MODBUS_RTU_COMM_CRC_ERROR;
  }

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
  _dbg.last_wait_first_us = ctx.first_byte_us
                                ? elapsed_us_or_zero(ctx.wait_start_us, ctx.first_byte_us)
                                : elapsed_us_or_zero(ctx.wait_start_us, MbusPlatform::microsNow());
  _dbg.last_rx_first_us = ctx.first_byte_us;
  _dbg.last_rx_last_us = ctx.last_byte_us;
  _dbg.last_read_total_us = ctx.last_byte_us
                                ? elapsed_us_or_zero(ctx.wait_start_us, ctx.last_byte_us)
                                : 0U;
  _dbg.last_rx_len = 0;
  _dbg.last_err = outErr;
  switch (outErr) {
    case MODBUS_RTU_COMM_TIMEOUT:
      _dbg.timeout_count++;
      if (ctx.saw_rx_activity) {
        _dbg.timeout_with_rx_count++;
      } else {
        _dbg.timeout_no_rx_count++;
      }
      if (ctx.saw_frame_err || ctx.saw_crc_err) {
        _dbg.timeout_with_parse_err_count++;
      }
      break;
    case MODBUS_RTU_COMM_FRAME_ERROR:
      _dbg.frame_err_count++;
      break;
    case MODBUS_RTU_COMM_CRC_ERROR:
      _dbg.crc_err_count++;
      break;
    default:
      break;
  }
#endif

  return outErr;
}

bool ModbusRTUComm::writeAdu(ModbusADU& adu) {
  if (!_drainToIdle()) {
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
    _dbg.last_err = MODBUS_RTU_COMM_FRAME_ERROR;
    _dbg.frame_err_count++;
#endif
    return false;
  }

  // Enforce inter-frame silence before transmitting.
  unsigned long now = MbusPlatform::microsNow();
  const unsigned long gateStart = now;
  const unsigned long nowMs = MbusPlatform::millisNow();

  // If we have been idle for a while, clear stale gates.
  const uint32_t lastTrafficMs = _lastTrafficMs.load(MBUS_MEM_RELAXED);
  if (lastTrafficMs != 0 && static_cast<uint32_t>(nowMs - lastTrafficMs) > _idleResetMs) {
    _replaceTxGate(now, 0U);
  }

  unsigned long remaining = _remainingTxGateUs(now);
  while (remaining != 0U) {
    if (remaining > 12000UL) {
      MbusPlatform::sleepMilliseconds(1);
    } else {
      _yield_if_long_gap(gateStart);
    }
    now = MbusPlatform::microsNow();
    remaining = _remainingTxGateUs(now);
  }

#if MBUS_RTU_PLATFORM_TRACE
  MbusPlatform::trace(MbusPlatformTraceRecord(
      MbusPlatformTraceEvent::TxGateOpen,
      static_cast<uint32_t>(now)));
#endif

  adu.updateCrc();
  const uint16_t txLen = adu.getRtuLen();
  const uint32_t postGapUs = static_cast<uint32_t>(_oneShotPostGapUs);
  _oneShotPostGapUs = 0;

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
  const unsigned long txStartUs = MbusPlatform::microsNow();
  _dbg.last_tx_wait_us = txStartUs - gateStart;
  _dbg.last_tx_start_us = txStartUs;
  _dbg.last_tx_len = txLen;
  _dbg.last_tx_id = adu.getUnitId();
  _dbg.last_tx_fc = adu.getFunctionCode();
#endif

#ifdef MBUS_PRINTALL
  _dumpFrame("TX ▶", adu.rtu, txLen);
#endif

  MbusPlatform::setDriverTransmit(_dePin, true);

#if MBUS_RTU_PLATFORM_TRACE
  MbusPlatform::trace(MbusPlatformTraceRecord(
      MbusPlatformTraceEvent::TxWriteStarted,
      MbusPlatform::microsNow(),
      txLen));
#endif
  const size_t written = MbusPlatform::write(_serial, adu.rtu, txLen);
  const bool flushed = MbusPlatform::waitForTransmitDrain(
      _serial, written, _charTimeUs);
  MbusPlatform::waitDelayMicroseconds(_postDelay);

  MbusPlatform::setDriverTransmit(_dePin, false);
#if MBUS_RTU_PLATFORM_TRACE
  MbusPlatform::trace(MbusPlatformTraceRecord(
      MbusPlatformTraceEvent::TxWriteFinished,
      MbusPlatform::microsNow(),
      (static_cast<uint32_t>(written) << 1U) | (flushed ? 1U : 0U)));
#endif

  if (written != txLen || !flushed) {
    _lastTrafficMs.store(MbusPlatform::millisNow(), MBUS_MEM_RELAXED);
    const uint32_t failureGateStartUs = MbusPlatform::microsNow();
    _replaceTxGate(
        failureGateStartUs,
        static_cast<uint64_t>(_frameTimeout) + _baseExtraGapUs + postGapUs);
#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
    _dbg.last_err = MODBUS_RTU_COMM_FRAME_ERROR;
    _dbg.frame_err_count++;
    _dbg.last_tx_done_us = MbusPlatform::microsNow();
#endif
    return false;
  }

  _lastTrafficMs.store(MbusPlatform::millisNow(), MBUS_MEM_RELAXED);
  _txSeq++;

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
  _dbg.last_tx_done_us = MbusPlatform::microsNow();
#endif

  uint32_t gateUs = _frameTimeout;
  bool noReply = false;
  uint32_t procUs = 0;

  const uint8_t unit = adu.getUnitId();
  const uint8_t fc = adu.getFunctionCode();

  if (unit == 0 && (fc == 5 || fc == 6 || fc == 15 || fc == 16)) {
    noReply = true;
    if (fc == 5 || fc == 6) {
      procUs = 200;
    } else if (fc == 16) {
      procUs = 350 + 8U * adu.getDataRegister(2);
    } else {
      procUs = 400 + 4U * adu.getDataRegister(2);
    }
  } else if (fc == 0x45) {
    noReply = true;
    const uint8_t innerFc = adu.data[1];
    if (innerFc == 5 || innerFc == 6) {
      procUs = 200;
    } else if (innerFc == 16 || innerFc == 15) {
      if (adu.getDataLen() >= 6) {
        const uint16_t qty = static_cast<uint16_t>(adu.data[4] << 8) | adu.data[5];
        if (innerFc == 16) {
          procUs = 350 + 8U * qty;
        } else {
          procUs = 400 + 4U * qty;
        }
      } else {
        procUs = 500;
      }
    }
  }

  if (noReply) {
    gateUs = max_val(gateUs, procUs);

    gateUs = max_val<uint32_t>(gateUs, _broadcastGapUs);
    if (gateUs > _noReplyCapUs) {
      gateUs = _noReplyCapUs;
    }
    const uint64_t completeGateUs =
        static_cast<uint64_t>(gateUs) + _baseExtraGapUs + postGapUs;
    _replaceTxGate(MbusPlatform::microsNow(), completeGateUs);
  } else {
    _replaceTxGate(
        MbusPlatform::microsNow(),
        static_cast<uint64_t>(_frameTimeout) + _baseExtraGapUs + postGapUs);
  }

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
  _dbg.extra_gap_us = _baseExtraGapUs + postGapUs;
#endif

  return true;
}

void ModbusRTUComm::_yield_if_long_gap(unsigned long start) {
  if ((MbusPlatform::microsNow() - start) > yieldThreshold) {
    MbusPlatform::yieldTask();
  }
}

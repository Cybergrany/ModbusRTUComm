// Modbus RTU transport API.
// Derived-source attribution is covered by the repository MIT license.
#ifndef ModbusRTUComm_h
#define ModbusRTUComm_h

#include "Arduino.h"
#include "ModbusADU.h"
#include "ModbusRTUDiagnosticsPolicy.h"

// Public capability level for higher-level Modbus libraries that can also
// build against the historical CMB27 compatibility seed. Capability checks
// avoid guessing from a repository version or an application-specific macro.
#define MBUS_RTU_COMM_COMPAT_API_VERSION 1
#define MBUS_RTU_COMM_HAS_ONE_SHOT_GAPS 1
// writeAdu() recognizes established no-response operations (unit-zero standard
// writes and FC69), completes without readAdu(), and holds the following TX
// behind the calculated turnaround gate.
#define MBUS_RTU_COMM_HAS_NO_RESPONSE_GATE 1
// Next-TX holdoffs are represented as bounded elapsed-time intervals rather
// than signed comparisons against an absolute micros() deadline.  Consumers
// can use this capability when they need to distinguish the long-idle-safe
// scheduler from older compatible transports.
#define MBUS_RTU_COMM_HAS_WRAP_SAFE_TX_GATE 1

#ifndef MASTER_RTU_EXTRA_GAP_US
#define MASTER_RTU_EXTRA_GAP_US 0UL
#endif

#define MBUS_RTU_TIMING_CALCULATED 0
#define MBUS_RTU_TIMING_SPEC_FIXED_GT19200 1

#ifndef MBUS_RTU_TIMING_MODE
// The Modbus Serial Line guide recommends fixed T1.5/T3.5 timers above
// 19200 baud.  Keep the calculated mode available for explicitly managed
// legacy networks, but make the standards-based timing the safe default.
#define MBUS_RTU_TIMING_MODE MBUS_RTU_TIMING_SPEC_FIXED_GT19200
#endif

#ifndef MBUS_RTU_T15_OVERRIDE_US
// Optional T1.5 override (microseconds). 0 keeps timing-mode/default behavior.
#define MBUS_RTU_T15_OVERRIDE_US 0UL
#endif

#ifndef MBUS_RTU_T15_DEFAULT_GT19200_US
// Deprecated compatibility override.  Fixed high-baud timing is now selected
// by MBUS_RTU_TIMING_MODE; leave this at zero unless preserving an old build.
#define MBUS_RTU_T15_DEFAULT_GT19200_US 0UL
#endif

#ifndef MBUS_RTU_T15_FIXED_GT19200_US
#define MBUS_RTU_T15_FIXED_GT19200_US 750UL
#endif

#ifndef MBUS_RTU_T35_FIXED_GT19200_US
#define MBUS_RTU_T35_FIXED_GT19200_US 1750UL
#endif

#ifndef SPEC_BROADCAST_GAP_US
#define SPEC_BROADCAST_GAP_US 150000UL
#endif

#ifndef MBUS_RTU_ENABLE_RX_THREAD
#define MBUS_RTU_ENABLE_RX_THREAD 0
#endif

#ifndef MBUS_RTU_RX_THREAD_STACK_WATERMARK_DIAG
// Enables RX worker stack watermark telemetry (min free / max used).
// Keep OFF in production; enable during stack sizing/profiling.
#define MBUS_RTU_RX_THREAD_STACK_WATERMARK_DIAG 0
#endif

#if MBUS_RTU_RX_THREAD_STACK_WATERMARK_DIAG && MBUS_RTU_ENABLE_RX_THREAD && (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
  #define MBUS_RTU_RX_THREAD_STACK_WATERMARK_ACTIVE 1
  #define MBUS_RTU_COMM_HAS_RX_STACK_SNAPSHOT 1
#else
  #define MBUS_RTU_RX_THREAD_STACK_WATERMARK_ACTIVE 0
#endif

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
#define MBUS_RTU_COMM_HAS_DEBUG_INFO 1
#endif

#ifndef MBUS_RTU_RX_THREAD_STACK_BYTES
#define MBUS_RTU_RX_THREAD_STACK_BYTES 2048U
#endif

#if MBUS_RTU_ENABLE_RX_THREAD && (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
  #include <atomic>
  using MbusAtomicU32 = std::atomic<uint32_t>;
  using MbusAtomicBool = std::atomic<bool>;
  static constexpr std::memory_order MBUS_MEM_RELAXED = std::memory_order_relaxed;
  static constexpr std::memory_order MBUS_MEM_ACQUIRE = std::memory_order_acquire;
  static constexpr std::memory_order MBUS_MEM_RELEASE = std::memory_order_release;
#else
  enum MbusMemoryOrder : uint8_t {
    MBUS_MEM_RELAXED = 0,
    MBUS_MEM_ACQUIRE = 1,
    MBUS_MEM_RELEASE = 2
  };

  class MbusAtomicU32 {
    public:
      constexpr MbusAtomicU32(uint32_t initial = 0) : _value(initial) {}
      inline uint32_t load(MbusMemoryOrder = MBUS_MEM_RELAXED) const { return _value; }
      inline void store(uint32_t value, MbusMemoryOrder = MBUS_MEM_RELAXED) { _value = value; }
      inline uint32_t fetch_add(uint32_t value, MbusMemoryOrder = MBUS_MEM_RELAXED) {
        const uint32_t old = _value;
        _value += value;
        return old;
      }

    private:
      uint32_t _value;
  };

  class MbusAtomicBool {
    public:
      constexpr MbusAtomicBool(bool initial = false) : _value(initial) {}
      inline bool load(MbusMemoryOrder = MBUS_MEM_RELAXED) const { return _value; }
      inline void store(bool value, MbusMemoryOrder = MBUS_MEM_RELAXED) { _value = value; }

    private:
      bool _value;
  };
#endif

#ifndef MBUS_RTU_RX_RING_SIZE
  #if defined(ARDUINO_ARCH_MBED) || defined(__MBED__)
    #define MBUS_RTU_RX_RING_SIZE 512
  #else
    #define MBUS_RTU_RX_RING_SIZE 256
  #endif
#endif

#ifndef MBUS_RTU_DEDUPE_WINDOW_US
#define MBUS_RTU_DEDUPE_WINDOW_US 4000UL
#endif

#ifndef MBUS_RTU_LATE_GRACE_US_MIN
#define MBUS_RTU_LATE_GRACE_US_MIN 2000UL
#endif

#ifndef MBUS_RTU_LATE_GRACE_US_MAX
#define MBUS_RTU_LATE_GRACE_US_MAX 8000UL
#endif

#ifndef MBUS_RTU_TRACE_STATES
// Diagnostic-only state transition trace. Keep disabled in production unless
// actively investigating transaction sequencing.
#define MBUS_RTU_TRACE_STATES 0
#endif

#if !MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED
#undef MBUS_RTU_TRACE_STATES
#define MBUS_RTU_TRACE_STATES 0
#ifdef MBUS_PRINTALL
#undef MBUS_PRINTALL
#endif
#endif

#ifndef MBUS_RTU_STATE_ASSERTS
  // Internal safety asserts for transport state invariants.
  //
  // Default is intentionally OFF for firmware builds to avoid assert-driven
  // resets in profiling/field runs. Enable explicitly when validating locally.
  // Native/unit-test builds can opt in via build flags.
  #define MBUS_RTU_STATE_ASSERTS 0
#endif

#ifndef MBUS_RTU_PROCESS_BURST_CAP
// Maximum number of consecutively processed extracted frames before yielding in
// PROCESS_FRAME. Prevents long parser loops from starving cooperative schedulers
// when large bursts/garbage are present.
#define MBUS_RTU_PROCESS_BURST_CAP 8
#endif

#ifndef MBUS_DIAG_NONBLOCKING_LOG
// When enabled, diagnostic/frame prints never block transport/runtime flow.
// Logs are dropped if the serial lock is contended or TX space is unavailable.
#define MBUS_DIAG_NONBLOCKING_LOG 1
#endif

#ifndef MBUS_RTU_DRAIN_ESCAPE_US
// Hard escape for RX drain loops. Prevents unbounded stalls when line noise or
// transport state never reaches idle.
#define MBUS_RTU_DRAIN_ESCAPE_US 250000UL
#endif

// #define MBUS_PRINTALL

enum ModbusRTUCommError : uint8_t {
  MODBUS_RTU_COMM_SUCCESS = 0,
  MODBUS_RTU_COMM_TIMEOUT = 1,
  MODBUS_RTU_COMM_FRAME_ERROR = 2,
  MODBUS_RTU_COMM_CRC_ERROR = 3
};

// Shared diagnostic serial lock used by Modbus frame prints and bridge runtime
// logs when threaded execution is enabled. The disabled policy is entirely
// stateless: it emits no mutex, BSS, or externally linked helper functions.
#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED
void mbusDiagSerialLock();
void mbusDiagSerialUnlock();
bool mbusDiagSerialTryLock();
bool mbusDiagSerialCanWrite(size_t bytesHint);
#else
static inline void mbusDiagSerialLock() noexcept {}
static inline void mbusDiagSerialUnlock() noexcept {}
static inline bool mbusDiagSerialTryLock() noexcept { return false; }
static inline bool mbusDiagSerialCanWrite(size_t) noexcept { return false; }
#endif

class ModbusRTUComm {
  public:
    ModbusRTUComm(Stream& serial, int8_t dePin = -1, int8_t rePin = -1);
    ~ModbusRTUComm();

    void begin(unsigned long baud, uint32_t config = SERIAL_8N1);
    // Response-start timeout in milliseconds. readAdu() bounds extreme values
    // internally so its complete timeout + late + frame window remains safe
    // across micros() rollover.
    void setTimeout(unsigned long timeout);
    // One-shot pre-transmit gap (in microseconds) for the next transaction.
    void setPreTxGapUsOnce(unsigned long extraGapUs);
    // One-shot post-transmit gap (in microseconds) applied after the next write.
    void setPostTxGapUsOnce(unsigned long extraGapUs);
    ModbusRTUCommError readAdu(ModbusADU& adu);
    bool writeAdu(ModbusADU& adu);
#if MBUS_RTU_RX_THREAD_STACK_WATERMARK_ACTIVE
    struct RxThreadStackSnapshot {
      uint32_t sizeBytes = 0;
      uint32_t freeNowBytes = 0;
      uint32_t minFreeBytes = 0;
      uint32_t maxUsedBytes = 0;
      uint32_t samples = 0;
    };
    RxThreadStackSnapshot rxThreadStackSnapshot() const;
#endif

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
    struct DebugInfo {
      uint32_t read_timeout_ms = 0;
      uint32_t char_time_us = 0;
      uint32_t char_timeout_us = 0;
      uint32_t frame_timeout_us = 0;
      uint32_t post_delay_us = 0;
      uint32_t extra_gap_us = 0;

      uint32_t last_tx_start_us = 0;
      uint32_t last_tx_done_us = 0;
      uint32_t last_tx_wait_us = 0;
      uint16_t last_tx_len = 0;
      uint8_t last_tx_id = 0;
      uint8_t last_tx_fc = 0;

      uint32_t last_read_start_us = 0;
      uint32_t last_wait_first_us = 0;
      uint32_t last_rx_first_us = 0;
      uint32_t last_rx_last_us = 0;
      uint32_t last_read_total_us = 0;
      uint16_t last_rx_len = 0;
      ModbusRTUCommError last_err = MODBUS_RTU_COMM_SUCCESS;

      uint32_t ok_count = 0;
      uint32_t timeout_count = 0;
      uint32_t frame_err_count = 0;
      uint32_t crc_err_count = 0;
      // Timeout classification counters for root-cause analysis.
      uint32_t timeout_no_rx_count = 0;
      uint32_t timeout_with_rx_count = 0;
      uint32_t timeout_with_parse_err_count = 0;
      uint32_t late_match_after_timeout_count = 0;

      uint32_t recovered_duplicate_count = 0;
      uint32_t recovered_stray_count = 0;
      uint32_t recovered_late_count = 0;
      uint32_t recovered_t15_count = 0;
      uint32_t recovered_parse_count = 0;
      uint32_t recovered_drain_drop_count = 0;
      uint32_t rx_overflow_count = 0;
      uint32_t drain_escape_count = 0;
      uint32_t last_drain_escape_us = 0;
      uint16_t last_drain_escape_rx_count = 0;
      uint32_t transaction_escape_count = 0;
      uint32_t last_transaction_escape_us = 0;
      uint32_t last_transaction_escape_budget_us = 0;
      uint16_t last_transaction_escape_rx_count = 0;
      uint8_t last_transaction_escape_state = 0;

      // State machine diagnostics (C2+ refactor instrumentation).
      uint32_t state_transition_count = 0;
      uint8_t state_last = 0;
      uint8_t state_current = 0;

#if MBUS_RTU_RX_THREAD_STACK_WATERMARK_DIAG
      uint32_t rx_thread_stack_size_bytes = 0;
      uint32_t rx_thread_stack_last_free_bytes = 0;
      uint32_t rx_thread_stack_min_free_bytes = 0;
      uint32_t rx_thread_stack_max_used_bytes = 0;
      uint32_t rx_thread_stack_samples = 0;
#endif
    };

    const DebugInfo& debugInfo() const;
#endif

  private:
    struct RxEntry {
      uint8_t byte = 0;
      uint32_t ts_us = 0;
    };

    struct FrameView {
      uint8_t data[256]{};
      uint16_t len = 0;
      uint32_t first_us = 0;
      uint32_t last_us = 0;
      bool t15_violation = false;
    };

    struct TimedOutPending {
      bool valid = false;
      uint8_t unit = 0;
      uint8_t fc = 0;
      uint16_t word0 = 0;
      uint16_t word2 = 0;
      uint8_t byte_count = 0;
      uint32_t grace_until_us = 0;
    };

    enum class ParseLenStatus : uint8_t {
      NeedMore = 0,
      HaveLen,
      Impossible
    };

    enum class ExtractStatus : uint8_t {
      None = 0,
      Ok,
      CrcError,
      FrameError,
      GapViolation
    };

    enum class RxTxnState : uint8_t {
      // Waiting for first meaningful byte/frame from the expected transaction.
      WAIT_FIRST = 0,
      // RX activity observed; transaction is bounded by RTU framing and a
      // defensive maximum legal-frame envelope.
      RX_ACTIVE,
      // A single extracted frame/error token is being classified.
      PROCESS_FRAME,
      // First-byte timeout occurred; we still allow a short late acceptance window.
      LATE_WINDOW,
      // Mandatory drain-to-idle barrier before request completion/retry.
      DRAIN,
      // Transaction complete (success or terminal failure).
      DONE
    };

    struct RxTxnCtx {
      // Current state and target return state when PROCESS_FRAME completes.
      RxTxnState state = RxTxnState::WAIT_FIRST;
      RxTxnState resume_after_process = RxTxnState::WAIT_FIRST;
      // Single-frame handoff payload produced by extractor.
      FrameView pending_frame{};
      ExtractStatus pending_extract = ExtractStatus::None;

      // Timing model (all absolute microsecond deadlines).
      uint32_t start_us = 0;
      uint32_t wait_start_us = 0;
      uint32_t read_timeout_us = 0;
      uint32_t first_byte_deadline_us = 0;
      uint32_t recovery_deadline_us = 0;
      uint32_t frame_envelope_deadline_us = 0;
      uint32_t late_deadline_us = 0;

      // Transaction outcome flags; these map directly to final comm error/result.
      bool saw_rx_activity = false;
      bool saw_match = false;
      bool saw_frame_err = false;
      bool saw_crc_err = false;
      bool late_window = false;
      bool late_accepted = false;
      bool did_drain = false;

      uint32_t first_byte_us = 0;
      uint32_t last_byte_us = 0;
      // Cooperative fairness limiter in PROCESS_FRAME.
      uint8_t burst_budget = MBUS_RTU_PROCESS_BURST_CAP;
    };

    const unsigned long _broadcastGapUs = 0;
    const unsigned long _baseExtraGapUs = static_cast<unsigned long>(MASTER_RTU_EXTRA_GAP_US);
    const unsigned long _noReplyCapUs = 6000; // cap fire-and-forget delay in legacy mode
    const unsigned long _idleResetMs = 5000;  // watchdog: clear stale TX gate after long idle
    unsigned long _charTimeUs = 0;
    // Defensive cap for one legal 256-byte RTU frame. Normal completion and
    // abandoned-frame handling are driven by T1.5/T3.5, not by this cap.
    unsigned long _maxFrameReceiveUs = 0;

    Stream& _serial;
    int8_t _dePin;
    int8_t _rePin;
    unsigned long _charTimeout = 0; // T1.5
    unsigned long _frameTimeout = 0; // T3.5
    unsigned long _postDelay = 0;
    unsigned long _readTimeout = 0;
    unsigned long _oneShotPostGapUs = 0;

    // The next-TX holdoff is an elapsed-time interval, not an absolute micros()
    // deadline. A coarse gate-start timestamp lets the scheduler retire an
    // interval before RX ingestion even after a complete micros() wrap.
    // Durations are bounded below the signed half-range; normal RTU/FC69 gates
    // are only a few milliseconds.
    uint32_t _txGateStartedUs = 0;
    uint32_t _txGateStartedMs = 0;
    uint32_t _txGateDurationUs = 0;
    // Last RX/TX activity time (millis) for idle watchdog recovery.
    // Updated by RX ingress thread and transport thread.
    MbusAtomicU32 _lastTrafficMs{0};

    // RX ingress ring (single-producer/single-consumer, lock-free indices).
    RxEntry _rxRing[MBUS_RTU_RX_RING_SIZE > 0 ? MBUS_RTU_RX_RING_SIZE : 1]{};
    MbusAtomicU32 _rxHead{0};
    MbusAtomicU32 _rxTail{0};
    MbusAtomicU32 _rxOverflowCount{0};
    MbusAtomicBool _rxReadableHint{false};
    MbusAtomicU32 _lastRxByteUs{0};

    // Parser scratch to avoid per-call stack spikes in extract/read hot paths.
    uint8_t _extractScratchBytes[256]{};
    uint32_t _extractScratchTs[256]{};

    // Transaction + dedupe history
    uint32_t _txSeq = 0;
    uint32_t _lastAcceptedTxSeq = 0;
    uint8_t _lastAcceptedFrame[256]{};
    uint16_t _lastAcceptedLen = 0;
    uint32_t _lastAcceptedUs = 0;

    // helper: yields after the first 100 us of silence
    static const unsigned long YIELD_MIN_US = 100;
    unsigned long yieldThreshold = YIELD_MIN_US;
    void _yield_if_long_gap(unsigned long start);

    void _attachIngressAdapter();
    void _detachIngressAdapter();
    void _startRxThreadIfEnabled();
    void _stopRxThread();
    void _wakeRxThread();
    void _onReadableSignal();
    static void _serialReadableThunk(void* ctx);
#if MBUS_RTU_ENABLE_RX_THREAD && (defined(ARDUINO_ARCH_MBED) || defined(__MBED__))
    static void _rxThreadThunk(void* ctx);
#endif
    void _rxThreadMain();
#if MBUS_RTU_RX_THREAD_STACK_WATERMARK_ACTIVE
    void _sampleRxThreadStack();
#endif

    bool _rxCallbackAttached = false;
    bool _rxThreadRunning = false;
    // Opaque platform event-task handle. The neutral Modbus contract and this
    // public header never expose an RTOS task/semaphore type.
    void* _rxThreadState = nullptr;

    void _ingestPoll();
    bool _pushRxByte(uint8_t byte, uint32_t tsUs);
    bool _popRxByte(uint8_t& byteOut, uint32_t& tsUsOut);
    bool _peekRx(uint16_t offset, RxEntry& out) const;
    uint16_t _rxCount() const;
    bool _rxEmpty() const;
    void _dropRxBytes(uint16_t n);

    ParseLenStatus _expectedFrameLength(const uint8_t* buf, uint16_t have, uint16_t& expectedLen) const;
    ExtractStatus _extractNextFrame(FrameView& frame);
    uint16_t _crc16(const uint8_t* buf, uint16_t lenWithoutCrc) const;

    bool _frameMatchesRequest(const FrameView& frame, uint8_t expectedUnit, uint8_t expectedFc) const;
    bool _isRecentDuplicate(const FrameView& frame) const;
    void _rememberAcceptedFrame(const FrameView& frame);

    void _armTimedOutPending(ModbusADU& reqAdu, uint32_t nowUs, uint32_t graceUs);
    void _expireTimedOutPending(uint32_t nowUs);
    bool _matchAndConsumeTimedOutPending(const FrameView& frame, uint32_t nowUs);

    bool _drainToIdle();
    uint32_t _computeLateGraceUs() const;
    static uint32_t _boundedTxGateUs(uint64_t requestedUs);
    uint32_t _remainingTxGateUs(uint32_t nowUs) const;
    void _replaceTxGate(uint32_t nowUs, uint64_t durationUs);
    void _extendTxGate(uint32_t nowUs, uint64_t durationUs);
    void _expireTxGateByCoarseAge(uint32_t nowMs);

    TimedOutPending _timedOutPending{};

#if defined(MBUS_DETAILED_METRICS) && MBUS_DETAILED_METRICS
    void _syncRxThreadStackDiagDebug() const;
    mutable DebugInfo _dbg{};
#endif

#if MBUS_RTU_RX_THREAD_STACK_WATERMARK_ACTIVE
    MbusAtomicU32 _rxThreadStackSizeBytes{0};
    MbusAtomicU32 _rxThreadStackLastFreeBytes{0};
    MbusAtomicU32 _rxThreadStackMinFreeBytes{0};
    MbusAtomicU32 _rxThreadStackSamples{0};
#endif

    #ifdef MBUS_PRINTALL
    void _dumpFrame(const char* label, const uint8_t* buf, uint16_t len){
#if MBUS_DIAG_NONBLOCKING_LOG
      if (!mbusDiagSerialTryLock()) {
        return;
      }
      if (!Serial) {
        mbusDiagSerialUnlock();
        return;
      }
      int avail = Serial.availableForWrite();
      if (avail <= 0) {
        mbusDiagSerialUnlock();
        return;
      }
      size_t labelLen = 0U;
      if (label) {
        for (const char* p = label; *p; ++p) {
          ++labelLen;
        }
      }
      const size_t overhead = labelLen + 4U; // label + space + newline + slack
      if (static_cast<size_t>(avail) <= overhead) {
        mbusDiagSerialUnlock();
        return;
      }
      const size_t budget = static_cast<size_t>(avail) - overhead;
      const uint16_t maxFrameBytes = static_cast<uint16_t>(budget / 3U);
      const uint16_t printLen = (len <= maxFrameBytes) ? len : maxFrameBytes;
#else
      const uint16_t printLen = len;
      mbusDiagSerialLock();
#endif
      Serial.print(label);
      Serial.print(' ');
      for (uint16_t i = 0; i < printLen; i++) {
        if (buf[i] < 0x10) Serial.print('0');
        Serial.print(buf[i], HEX);
        Serial.print(' ');
      }
      if (printLen < len) {
        Serial.print("# ");
      }
      Serial.println();
      mbusDiagSerialUnlock();
    }
    #endif
};

#endif

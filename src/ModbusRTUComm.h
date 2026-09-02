#ifndef ModbusRTUComm_h
#define ModbusRTUComm_h

#include "Arduino.h"
#include "ModbusADU.h"

enum ModbusRTUCommError : uint8_t {
  MODBUS_RTU_COMM_SUCCESS = 0,
  MODBUS_RTU_COMM_TIMEOUT = 1,
  MODBUS_RTU_COMM_FRAME_ERROR = 2,
  MODBUS_RTU_COMM_CRC_ERROR = 3
};

// Returns true when the bytes received so far are a complete structural frame
// candidate. The communication layer still verifies the CRC before accepting
// it. One callback may nominate any number of custom function/length shapes.
typedef bool (*ModbusRTUFrameCandidateFn)(const uint8_t* rtu,
                                          uint16_t receivedLen);

// Recovery from a continuous malformed stream is deliberately bounded per
// readAdu() call. Override these before including the header if a platform
// needs different cooperative-loop limits.
#ifndef MODBUS_RTU_DRAIN_MAX_BYTES_PER_PASS
#define MODBUS_RTU_DRAIN_MAX_BYTES_PER_PASS 512U
#endif

#ifndef MODBUS_RTU_DRAIN_MAX_MICROS_PER_PASS
#define MODBUS_RTU_DRAIN_MAX_MICROS_PER_PASS 20000UL
#endif

class ModbusRTUComm {
  public:
    ModbusRTUComm(Stream& serial, int dePin = -1, int rePin = -1);
    void begin(unsigned long baud, uint32_t config = SERIAL_8N1);
    void setTimeout(unsigned long timeout);
    ModbusRTUCommError readAdu(ModbusADU& adu);
    // T3.5 remains the authoritative frame boundary. If multiple frames were
    // already queued before they could be serviced, their historical gaps are
    // no longer observable; CRC-valid structural candidates are then used to
    // preserve the trailing bytes for the next read. The one-argument reader
    // remains unchanged.
    ModbusRTUCommError readAdu(ModbusADU& adu,
                               ModbusRTUFrameCandidateFn frameCandidate,
                               ModbusRTUFrameCandidateFn additionalFrameCandidate,
                               bool& usedBufferedCandidate);
    bool writeAdu(ModbusADU& adu);

  private:
    Stream& _serial;
    int _dePin;
    int _rePin;
    unsigned long _charTimeout;
    unsigned long _frameTimeout;
    unsigned long _bytePeriod;
    unsigned long _postDelay;
    unsigned long _readTimeout = 0;
    bool _discardUntilIdle = false;
    unsigned long _discardLastByteMicros = 0;

    void beginDiscard(unsigned long lastByteMicros);
    void drainUntilIdle();

};

#endif

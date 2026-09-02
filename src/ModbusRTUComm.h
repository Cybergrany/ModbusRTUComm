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

// Returns the expected total RTU ADU length, including CRC, once enough of
// the frame has been received to determine it. A return value of zero means
// that the length is not known yet.
typedef uint16_t (*ModbusRTUExpectedLengthFn)(const uint8_t* rtu,
                                              uint16_t receivedLen);

class ModbusRTUComm {
  public:
    ModbusRTUComm(Stream& serial, int dePin = -1, int rePin = -1);
    void begin(unsigned long baud, uint32_t config = SERIAL_8N1);
    void setTimeout(unsigned long timeout);
    ModbusRTUCommError readAdu(ModbusADU& adu);
    // Stops at a CRC-valid expected length so queued trailing bytes remain for
    // the next read. When trailing bytes are already available, this overload
    // returns without an additional T3.5 wait so the caller can drain them;
    // otherwise it preserves the normal frame-gap wait. The one-argument
    // reader remains unchanged.
    ModbusRTUCommError readAdu(ModbusADU& adu,
                               ModbusRTUExpectedLengthFn expectedLength);
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

};

#endif

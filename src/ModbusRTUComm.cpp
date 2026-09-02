#include "ModbusRTUComm.h"

ModbusRTUComm::ModbusRTUComm(Stream& serial, int dePin, int rePin) : _serial(serial) {
  _dePin = dePin;
  _rePin = rePin;
}

void ModbusRTUComm::begin(unsigned long baud, uint32_t config) {
  _discardUntilIdle = false;
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
  if (baud <= 19200) {
    _charTimeout = (bitsPerChar * 2500000) / baud;
    _frameTimeout = (bitsPerChar * 4500000) / baud;
  }
  else {
    _charTimeout = (bitsPerChar * 1000000) / baud + 750;
    _frameTimeout = (bitsPerChar * 1000000) / baud + 1750;
  }
  _bytePeriod = (bitsPerChar * 1000000) / baud;
  _postDelay = ((bitsPerChar * 1000000) + 1500000) / baud;
  if (_dePin >= 0) {
    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
  }
  if (_rePin >= 0) {
    pinMode(_rePin, OUTPUT);
    digitalWrite(_rePin, LOW);
  }
  unsigned long startMicros = micros();
  do {
    if (_serial.available() > 0) {
      startMicros = micros();
      _serial.read();
    }
  } while (micros() - startMicros < _frameTimeout);
}

void ModbusRTUComm::setTimeout(unsigned long timeout) {
  _readTimeout = timeout;
}

ModbusRTUCommError ModbusRTUComm::readAdu(ModbusADU& adu) {
  adu.setRtuLen(0);
  unsigned long startMillis = millis();
  while (!_serial.available()) {
    if (millis() - startMillis >= _readTimeout) return MODBUS_RTU_COMM_TIMEOUT;
  }
  uint16_t len = 0;
  unsigned long startMicros = micros();
  do {
    if (_serial.available()) {
      startMicros = micros();
      adu.rtu[len] = _serial.read();
      len++;
    }
  } while (micros() - startMicros <= _charTimeout && len < 256);
  adu.setRtuLen(len);
  while (micros() - startMicros < _frameTimeout);
  if (_serial.available()) {
    adu.setRtuLen(0);
    return MODBUS_RTU_COMM_FRAME_ERROR;
  }
  if (!adu.crcGood()) {
    adu.setRtuLen(0);
    return MODBUS_RTU_COMM_CRC_ERROR;
  }
  return MODBUS_RTU_COMM_SUCCESS;
}

void ModbusRTUComm::beginDiscard(unsigned long lastByteMicros) {
  _discardUntilIdle = true;
  _discardLastByteMicros = lastByteMicros;
}

void ModbusRTUComm::drainUntilIdle() {
  const unsigned long passStartMicros = micros();
  uint16_t drained = 0;
  while (_discardUntilIdle) {
    const unsigned long now = micros();
    if (now - _discardLastByteMicros >= _frameTimeout) {
      _discardUntilIdle = false;
      return;
    }
    if (drained >= MODBUS_RTU_DRAIN_MAX_BYTES_PER_PASS ||
        now - passStartMicros >= MODBUS_RTU_DRAIN_MAX_MICROS_PER_PASS) {
      return;
    }
    if (_serial.available()) {
      const int value = _serial.read();
      if (value >= 0) {
        _discardLastByteMicros = micros();
        drained++;
      }
    }
  }
}

ModbusRTUCommError ModbusRTUComm::readAdu(
    ModbusADU& adu,
    ModbusRTUFrameCandidateFn frameCandidate,
    ModbusRTUFrameCandidateFn additionalFrameCandidate,
    bool& usedBufferedCandidate) {
  adu.setRtuLen(0);
  usedBufferedCandidate = false;
  if (_discardUntilIdle) {
    drainUntilIdle();
    return MODBUS_RTU_COMM_FRAME_ERROR;
  }
  unsigned long startMillis = millis();
  while (!_serial.available()) {
    if (millis() - startMillis >= _readTimeout) return MODBUS_RTU_COMM_TIMEOUT;
  }
  uint16_t len = 0;
  unsigned long lastByteMicros = micros();
  while (true) {
    const unsigned long now = micros();
    const unsigned long idleMicros = now - lastByteMicros;

    // Check for T3.5 before looking at the receive queue. If this call kept up
    // with the wire, a newly queued byte after that gap belongs to the next ADU.
    if (len > 0 && idleMicros >= _frameTimeout) {
      adu.setRtuLen(len);
      if (!adu.crcGood()) {
        adu.setRtuLen(0);
        return MODBUS_RTU_COMM_CRC_ERROR;
      }
      return MODBUS_RTU_COMM_SUCCESS;
    }

    if (_serial.available()) {
      // A newly observed byte between T1.5 and T3.5 invalidates the complete
      // RTU frame. Consume that byte before entering persistent drain mode so
      // it cannot later be mistaken for the start of a fresh frame.
      if (len > 0 && idleMicros > _charTimeout) {
        const int value = _serial.read();
        if (value >= 0) lastByteMicros = micros();
        beginDiscard(lastByteMicros);
        drainUntilIdle();
        adu.setRtuLen(0);
        return MODBUS_RTU_COMM_FRAME_ERROR;
      }

      if (len >= 256U) {
        beginDiscard(lastByteMicros);
        drainUntilIdle();
        adu.setRtuLen(0);
        return MODBUS_RTU_COMM_FRAME_ERROR;
      }

      const int value = _serial.read();
      if (value < 0) continue;
      adu.rtu[len] = value;
      len++;
      lastByteMicros = micros();

      // Candidate parsing is a fallback only when more bytes are already
      // queued. A normally serviced frame therefore needs only its T3.5 gap,
      // and a live byte arriving before T3.5 is not split from its prefix.
      if (_serial.available() && len >= 4U &&
          ((frameCandidate && frameCandidate(adu.rtu, len)) ||
           (additionalFrameCandidate &&
            additionalFrameCandidate(adu.rtu, len)))) {
        adu.setRtuLen(len);
        if (adu.crcGood()) {
          usedBufferedCandidate = true;
          return MODBUS_RTU_COMM_SUCCESS;
        }
        adu.setRtuLen(0);
      }
    }
  }
}

bool ModbusRTUComm::writeAdu(ModbusADU& adu) {
  uint16_t i = 0;
  uint16_t j = 0;
  bool transmitting = true;
  bool verified = false;
  adu.updateCrc();
  uint16_t len = adu.getRtuLen();
  if (_dePin >= 0) digitalWrite(_dePin, HIGH);
  unsigned long microsNow= micros();
  unsigned long txStartMicros = microsNow;
  unsigned long rxStartMicros = microsNow;
  while (true) {
    microsNow = micros();
    if (transmitting) {
      if (i == 0 || (i < len && microsNow - txStartMicros >= _bytePeriod)) {
        txStartMicros = microsNow;
        _serial.write(adu.rtu[i]);
        _serial.flush();
        i++;
      }
      if (i == len && microsNow - txStartMicros >= _postDelay) {
        if (_dePin >= 0) digitalWrite(_dePin, LOW);
        transmitting = false;
      }
    }
    if (_serial.available()) {
      rxStartMicros = microsNow;
      uint8_t value = _serial.read();
      if (j == 0) verified = true;
      if (j < len && value != adu.rtu[j]) verified = false;
      j++;
    }
    if (!transmitting && (microsNow - rxStartMicros) > _charTimeout) {
      if (j != len) verified = false;
      break;
    }
  }
  return verified;
}

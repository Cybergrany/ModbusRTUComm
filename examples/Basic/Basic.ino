#include <Arduino.h>
#include <ModbusADU.h>
#include <ModbusRTUComm.h>

namespace {
constexpr unsigned long kBaud = 19200UL;
ModbusRTUComm transport(Serial);
}

void setup() {
  // The application owns serial configuration. begin() calculates RTU timing,
  // initializes optional direction pins, drains stale RX, and starts the
  // selected ingress backend; it does not call Serial.begin().
  Serial.begin(kBaud, SERIAL_8N1);
  transport.begin(kBaud, SERIAL_8N1);
  transport.setTimeout(250UL);
}

void loop() {
  // A higher-level Modbus master/slave normally prepares the request ADU and
  // interprets the response. The low-level transport API is shown explicitly.
  ModbusADU request;
  request.setUnitId(1);
  request.setFunctionCode(3);
  request.setDataRegister(0, 0);
  request.setDataRegister(2, 1);
  request.setDataLen(4);

  if (transport.writeAdu(request)) {
    (void)transport.readAdu(request);
  }
  delay(1000);
}

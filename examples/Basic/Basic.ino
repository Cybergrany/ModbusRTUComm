#include <Arduino.h>
#include <ModbusADU.h>
#include <ModbusRTUComm.h>

// This example deliberately reuses the default Serial endpoint as the bus.
// Disable direct serial diagnostics for the *whole build* (including the
// separately compiled library source), not only in this sketch translation
// unit. See examples/Basic/README.md.
#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED != 0
#error "Build Basic with -DMBUS_RTU_ALLOW_DIRECT_SERIAL_DIAGNOSTICS=0"
#endif

#if MBUS_RTU_COMM_HAS_NO_RESPONSE_GATE != 1
#error "This example requires the reviewed no-response/next-TX gate"
#endif

#if (defined(ARDUINO_ARCH_MBED) || defined(ARDUINO_GIGA)) && \
    MBUS_RTU_RX_RING_SIZE != 512
#error "The supported GIGA production profile uses a 512-entry RX ring"
#endif

namespace {
constexpr unsigned long kBaud = 19200UL;
ModbusRTUComm transport(Serial);
}

void setup() {
  // The application owns serial configuration. begin() calculates RTU timing,
  // initializes DE and optional RE low (receive-safe), drains stale RX, and
  // starts the selected ingress backend. Only DE toggles around each TX;
  // begin() does not call Serial.begin().
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

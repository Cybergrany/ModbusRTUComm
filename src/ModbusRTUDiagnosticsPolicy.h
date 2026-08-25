#ifndef MODBUS_RTU_DIAGNOSTICS_POLICY_H_
#define MODBUS_RTU_DIAGNOSTICS_POLICY_H_

// Compile-time selection point for optional direct-serial diagnostics.
//
// A policy header must define MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED to
// exactly 0 or 1. The compatibility policy remains the default, preserving the
// validated transport behavior. A consumer can inject its own
// header without teaching the Modbus transport about application-level flags:
//
//   -DMBUS_RTU_DIAGNOSTICS_POLICY_HEADER=\"my/Policy.h\"
//
// The selected policy is build configuration and must be consistent in every
// translation unit that includes ModbusRTUComm or a platform backend.
#ifndef MBUS_RTU_DIAGNOSTICS_POLICY_HEADER
#define MBUS_RTU_DIAGNOSTICS_POLICY_HEADER \
  "platform/DirectSerialDiagnosticsPolicy.h"
#endif

#include MBUS_RTU_DIAGNOSTICS_POLICY_HEADER

#ifndef MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED
#error "The selected Modbus RTU diagnostics policy must define MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED"
#endif

#if MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED != 0 && \
    MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED != 1
#error "MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED must be 0 or 1"
#endif

#endif  // MODBUS_RTU_DIAGNOSTICS_POLICY_H_

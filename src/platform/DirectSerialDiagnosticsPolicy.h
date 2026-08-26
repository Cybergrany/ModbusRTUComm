#ifndef MODBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_POLICY_H_
#define MODBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_POLICY_H_

// Default-on preserves the established compatibility transport behavior. A
// firmware whose default Serial endpoint carries framed traffic must set this
// to 0 for the whole dependency tree. This removes plaintext output only;
// Modbus recovery/error results remain active. Exact drain-escape counters
// require a separate MBUS_DETAILED_METRICS=1 diagnostic build.
#ifndef MBUS_RTU_ALLOW_DIRECT_SERIAL_DIAGNOSTICS
#define MBUS_RTU_ALLOW_DIRECT_SERIAL_DIAGNOSTICS 1
#endif

#if MBUS_RTU_ALLOW_DIRECT_SERIAL_DIAGNOSTICS
#define MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED 1
#else
#define MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED 0
#endif

#endif

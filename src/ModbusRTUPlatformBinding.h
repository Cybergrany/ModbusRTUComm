#ifndef MODBUS_RTU_PLATFORM_BINDING_H_
#define MODBUS_RTU_PLATFORM_BINDING_H_

// Compile-time binding between ModbusRTUComm and one static platform facade.
//
// Defaults preserve the production Arduino/GIGA implementation. Consumers
// preparing another package/platform may select both a header and a facade
// type without editing transport business logic:
//
//   -DMBUS_RTU_PLATFORM_HEADER=\"my/ModbusPlatform.h\"
//   -DMBUS_RTU_PLATFORM_TYPE=my_namespace::MyModbusPlatform
//
// The selected type must implement the StaticModbusRTUPlatform surface
// documented in platform/README.md. Selection is compile-time only: this adds
// no stored pointer, virtual dispatch, function table, or runtime branch.
#ifndef MBUS_RTU_PLATFORM_HEADER
#define MBUS_RTU_PLATFORM_HEADER \
  "platform/arduino/ArduinoModbusRTUPlatform.h"
#endif

#include MBUS_RTU_PLATFORM_HEADER

#ifndef MBUS_RTU_PLATFORM_TYPE
#define MBUS_RTU_PLATFORM_TYPE \
  modbus_rtu::platform::ArduinoModbusRTUPlatform
#endif

#endif  // MODBUS_RTU_PLATFORM_BINDING_H_

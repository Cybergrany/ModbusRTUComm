# ModbusRTUComm

`ModbusRTUComm` is a low-level Arduino transport for Modbus RTU. It sends and
receives `ModbusADU` frames, enforces RTU timing, controls RS485 direction
pins, and recovers from damaged or unrelated input.

It is not a complete Modbus master or slave. Most applications use it through
a protocol layer such as
[`ModbusRTUMaster`](https://github.com/Cybergrany/ModbusRTUMaster).

## When to use it

Use this library when you are:

- building a Modbus master or server around a custom protocol layer;
- driving a half-duplex RS485 transceiver with DE and optional RE pins;
- receiving bytes through a buffered or event-driven serial backend;
- handling broadcasts where the next transmission must wait for bus idle; or
- adding transport metrics without coupling protocol code to one platform.

## What this fork adds

- **Continuous buffered receive and recovery.** Timestamped ingress separates
  frames at T1.5/T3.5 boundaries and classifies late, duplicate, stray,
  overflowed, CRC-invalid, and malformed input. This helps long-running
  multidrop systems recover without restarting the transport. See
  [`ModbusRTUComm.h`](src/ModbusRTUComm.h) and the
  [test matrix](test/README.md).
- **Deterministic RS485 direction order.** Each send follows DE high, write,
  drain, post-delay, and DE low. This is useful with transceivers that must not
  release the bus before the final stop bit has cleared. See the
  [platform TX contract](src/platform/README.md#tx-ordering).
- **No-response and next-transmit gates.** Broadcast and fire-and-forget users
  can finish without reading a reply while still preserving the required bus
  turnaround delay.
- **One-shot scheduling gaps.** A caller can add a pre- or post-transmit gap to
  one transaction without changing normal RTU timing for the rest of the bus.
- **Static platform binding.** Clocks, delays, GPIO, drain handling, readable
  events, tasks, and trace hooks can be replaced at compile time without
  virtual dispatch or allocation. See the
  [platform API](src/platform/README.md) and
  [Arduino backend](src/platform/arduino/README.md).
- **Optional metrics and diagnostics.** Detailed counters, timing snapshots,
  trace hooks, and RX-task stack data are compiled only when requested. See
  [configuration](#configuration).

## Quick start

The application owns the serial port and must configure it before calling
`begin()`.

```cpp
#include <Arduino.h>
#include <ModbusADU.h>
#include <ModbusRTUComm.h>

constexpr unsigned long kBaud = 19200UL;
constexpr int8_t kDriverEnablePin = 2;

ModbusRTUComm transport(Serial1, kDriverEnablePin);

void setup() {
  Serial1.begin(kBaud, SERIAL_8N1);
  transport.begin(kBaud, SERIAL_8N1);
  transport.setTimeout(250UL);
}

ModbusRTUCommError readOneHoldingRegister(uint8_t unit, uint16_t address,
                                         uint16_t& value) {
  ModbusADU adu;
  adu.setUnitId(unit);
  adu.setFunctionCode(3);
  adu.setDataRegister(0, address);
  adu.setDataRegister(2, 1);
  adu.setDataLen(4);

  if (!transport.writeAdu(adu)) {
    return MODBUS_RTU_COMM_FRAME_ERROR;
  }

  const ModbusRTUCommError result = transport.readAdu(adu);
  if (result == MODBUS_RTU_COMM_SUCCESS) {
    value = adu.getDataRegister(1);
  }
  return result;
}
```

`writeAdu()` updates the CRC. It returns `false` if the frame cannot be fully
written or drained. `readAdu()` uses the unit and function already stored in
the ADU to identify the expected response.

## Result values

| Value | Meaning |
| --- | --- |
| `MODBUS_RTU_COMM_SUCCESS` | A matching, CRC-valid response was received. |
| `MODBUS_RTU_COMM_TIMEOUT` | No matching response completed before the timeout. |
| `MODBUS_RTU_COMM_FRAME_ERROR` | Framing, drain, or terminal receive recovery failed. |
| `MODBUS_RTU_COMM_CRC_ERROR` | The terminal candidate had an invalid CRC. |

## One-shot gaps

```cpp
transport.setPreTxGapUsOnce(5000UL);
transport.setPostTxGapUsOnce(3000UL);
transport.writeAdu(adu);
```

Each value is consumed by the next attempted transaction. Use this for a
specific slow device or transition, not as a replacement for correct bus-wide
RTU timing.

## Platform backends

The default backend uses Arduino clocks, delays, GPIO, and `Stream`. A build
can select another static backend:

```ini
build_flags =
  -DMBUS_RTU_PLATFORM_HEADER=\"my/ModbusPlatform.h\"
  -DMBUS_RTU_PLATFORM_TYPE=my::ModbusPlatform
```

The selected backend must implement the contract in
[`src/platform/README.md`](src/platform/README.md). Use the same selection in
every translation unit that includes the library.

## Configuration

Common whole-build options include:

- `MBUS_RTU_ALLOW_DIRECT_SERIAL_DIAGNOSTICS=0` when the diagnostics console
  shares the Modbus serial port;
- `MBUS_DETAILED_METRICS=1` for `debugInfo()`;
- `MBUS_RTU_ENABLE_RX_THREAD=1` when the selected backend supports an RX event
  task; and
- `MBUS_RTU_RX_RING_SIZE=<power-of-two>` to choose a larger ingress ring for a
  busy or high-latency system.

Options that affect class layout or behavior must be identical in the
application and the separately compiled library source.

## Examples and reference

- [Basic low-level transaction](examples/Basic/Basic.ino)
- [Example configuration notes](examples/Basic/README.md)
- [Platform backend contract](src/platform/README.md)
- [Arduino and GIGA/mbed backend](src/platform/arduino/README.md)
- [Public transport header](src/ModbusRTUComm.h)
- [Validation guide](test/README.md)

## Testing

The PlatformIO native environments cover production, trace, C++11, and
failure backends:

```sh
pio test -e native_trace_on
pio test -e native_trace_off
pio test -e native_cxx11
pio test -e native_failure_backend
pio test -e production_profile
```

Native tests verify logical timing and operation order, but not UART interrupt
latency or electrical RS485 turnaround. Validate those on each target.

## License

MIT. See [LICENSE](LICENSE).

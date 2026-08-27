# Modbus RTU platform contract

`ModbusRTUPlatform.h` is the platform-neutral boundary used by
`ModbusRTUComm`. It contains no Arduino, mbed, RTOS, application, or concrete
serial type. `StaticModbusRTUPlatform<Backend>` delegates to static backend
methods, allowing the compiler to inline the selected implementation without a
vtable, function table, allocation, or stored backend pointer.

The production binding is selected by `ModbusRTUPlatformBinding.h`:

```cpp
#define MBUS_RTU_PLATFORM_HEADER "my/FakeModbusPlatform.h"
#define MBUS_RTU_PLATFORM_TYPE my::FakeModbusPlatform
#include <ModbusRTUComm.h>
```

Define both selectors consistently for every translation unit that sees the
transport. The default is `platform/arduino/ArduinoModbusRTUPlatform.h` and its
`ArduinoModbusRTUPlatform` facade.

## Backend surface

A complete backend provides these static operations:

| Area | Operations and contract |
| --- | --- |
| Clock/wait | `microsNow()`, `millisNow()`, `sleepMilliseconds()`, `waitDelayMicroseconds()`, and `yieldTask()`. Each clock wraps independently at its natural `uint32_t` period; `millisNow()` must retain its epoch across a `microsNow()` wrap rather than being derived from an already-truncated microsecond value. Waits must not return before their logical delay. |
| Stream | `available()`, `read()`, `write()`, and `waitForTransmitDrain()`. A successful drain means accepted bytes are safe to follow with the caller's post-delay and DE-low transition. |
| RS485 direction | `configureDriverPins()` configures supplied DE and RE pins as outputs and drives both low (receive-safe); `setDriverTransmit()` performs only the requested DE transition. Negative pins are unused. RE is not toggled per transmission. |
| Readable hint | `attachReadable()` installs a short edge callback or returns false for polling. `detachReadable()` removes it. Neither operation owns the stream or context. |
| Event task | `startEventTask()`, `eventTaskRunning()`, `waitEvent()`, `signalEvent()`, and `stopEventTask()`. Signals issued before a wait must remain observable. |
| Diagnostics | Lock, try-lock, unlock, and capacity checks protect optional logging only; they never serialize Modbus traffic. |
| Stack/trace | `currentTaskStack()` returns an invalid snapshot if unsupported. `trace()` exists only when `MBUS_RTU_PLATFORM_TRACE=1`. |

`startEventTask()` has an important publication rule: on success the backend
must publish a non-null, signalable handle before making the entry point
runnable. On failure it must return false with a null handle, no runnable task,
and no retained resources. This lets a readable edge arriving during task
startup be replayed safely:

```cpp
void* task = nullptr;
const modbus_rtu::platform::EventTaskConfig config(
    modbus_rtu::platform::TaskPriority::AboveNormal, 768U, "mbus-rx");
if (Platform::startEventTask(task, config, &rxTask, context)) {
  Platform::replayPendingEvent(task, readableWasAlreadySignalled);
}
```

## Deadlines and wrapping clocks

`Deadline` uses signed-difference comparisons around an unsigned wrapping
microsecond clock. Each delay and observation interval must stay below the
signed half-range (`2^31` microseconds, about 35.8 minutes):

```cpp
const uint32_t now = Platform::microsNow();
const modbus_rtu::platform::Deadline idle =
    modbus_rtu::platform::Deadline::after(now, 1750U);
while (!idle.reached(Platform::microsNow())) {
  Platform::yieldTask();
}
```

Next-TX gates use a bounded microsecond elapsed interval for precise active
timing and the independent millisecond clock only to prove that a dormant gate
has expired after a complete microsecond-clock wrap. Backends that derive
`millisNow()` from the low 32 bits of `microsNow()` violate this contract and
cannot provide the long-idle guarantee.

## TX ordering

The transport owns the exact order for every attempted frame, including a
partial write. RE remains at the receive-safe level established by `begin()`;
it is deliberately absent from the per-frame sequence:

1. DE high.
2. Stream write and accepted-byte count.
3. Drain accepted bytes.
4. Post-TX delay.
5. DE low.

The native transport tests assert this call order, event timestamps, attempted
and accepted lengths, and cleanup after partial writes. A backend must not hide
an additional DE transition inside `write()` or `waitForTransmitDrain()`.

## Trace and diagnostics

Trace is characterization-only and defaults off. A trace-enabled backend may
forward `TraceRecord` through `MBUS_RTU_PLATFORM_TRACE_HOOK(record)`. The hook
runs synchronously and may be invoked from caller, callback, or task context;
it must be bounded, nonblocking, allocation-free, and safe under concurrency.
Trace builds make extra clock calls and are intentionally not timing-identical
to production builds.

Direct serial diagnostics are selected by
`MBUS_RTU_DIAGNOSTICS_POLICY_HEADER`. The default policy uses
`MBUS_RTU_ALLOW_DIRECT_SERIAL_DIAGNOSTICS` (default `1`) and defines the final
`MBUS_RTU_DIRECT_SERIAL_DIAGNOSTICS_ENABLED` value. Set the former to `0` for
firmware whose console carries framed traffic. The selection must be identical
in the application and the separately compiled transport source; a definition
local to a sketch or one source file is insufficient.

## Supported RX-ring profiles

`MBUS_RTU_RX_RING_SIZE` defaults to `256` entries on generic non-mbed builds
and `512` entries on the supported mbed/GIGA build. Both are power-of-two
profiles and are asserted by the production/native and embedded compile gates.

A non-mbed gateway or high-latency consumer that needs the larger profile can
select it explicitly for the whole build:

```ini
build_flags =
  -DMBUS_RTU_RX_RING_SIZE=512
```

## Validation boundary

Native tests characterize delegation, deadlines, event ordering, exact frame
bytes, parser state transitions, and trace-on/off behavior. They cannot prove
UART interrupt latency, scheduler jitter, shift-register empty timing, RS485
electrical idle, or callback/teardown races. Those remain target compile and
physical-bus release gates.

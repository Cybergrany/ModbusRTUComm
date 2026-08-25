# Arduino backend

`ArduinoModbusRTUPlatform.h` implements the neutral contract documented in
[`../README.md`](../README.md). Generic Arduino targets use the supplied
`Stream`, Arduino clocks/delays/GPIO, and cooperative polling. They return false
from readable-callback and event-task startup, and return an invalid stack
snapshot.

On Arduino GIGA/mbed the backend additionally:

- connects registered `GigaBufferedSerial` instances to `sigio` readable hints;
- creates an mbed thread plus semaphore behind an opaque `void*` handle;
- publishes that handle before thread start and destroys it on any start error;
- joins and releases task resources during stop;
- reports the current mbed task stack when the RTOS supplies valid values; and
- waits an accepted-byte wire-time estimate instead of calling
  `BufferedSerial::sync()`, which can wedge the worker used by the established
  firmware behavior.

The estimate is deliberately conservative but is not measurement of UART
shift-register empty or RS485 electrical idle. Physical turnaround remains a
hardware-validation item.

`GigaBufferedSerial` is registered by object identity. For an ordinary
`Stream`, the backend falls back to `flush()` and polling:

```cpp
GigaBufferedSerial bus(TX_PIN, RX_PIN);
bus.begin(250000, SERIAL_8N1);
ModbusRTUComm comm(bus, DE_PIN, RE_PIN);
comm.begin(250000, SERIAL_8N1);
```

This package owns both the global `GigaBufferedSerial` compatibility type and
its `GigaSerialFormat` helper. Its Arduino backend includes those headers by a
package-relative path so an application include directory cannot silently
substitute a different definition. OGM_Portable must remove or exclude its old
global copy when selecting this package; compiling both definitions into one
firmware image violates the C++ one-definition rule.

At `begin()`, supplied DE and RE pins are configured as outputs and driven low.
Only DE transitions around later transmissions; RE remains in its receive-safe
state and is not part of the per-frame transmit sequence.

Readable callbacks are hints only; they do not parse frames or own bytes. They
set transport state and signal the task, which later drains the stream through
`available()` and `read()`.

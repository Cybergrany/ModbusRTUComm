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
  `BufferedSerial::sync()`, which can wedge the worker used by the validated
  firmware.

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

Readable callbacks are hints only; they do not parse frames or own bytes. They
set transport state and signal the task, which later drains the stream through
`available()` and `read()`.

# Compatibility transport tests

The native suite is a deterministic characterization of the transport replay
from the immutable source tuple recorded in `ogm-fork-lock.json`. It uses a
fake wrapping clock and timestamped `Stream`, not wall-clock sleeps.

Coverage includes:

- exact FC03 and FC69 transmitted bytes and CRC order;
- successful writes without a transceiver-local echo;
- DE-high, write, drain, post-delay, and DE-low ordering for complete and
  partial writes;
- T1.5/T3.5 boundaries, maximum legal FC03 response timing, and one-frame TX
  operation budget;
- first-byte timeout, late grace, duplicate/stray recovery, CRC/frame error
  precedence, buffer cleanup, and micros rollover;
- no-response/broadcast gates and preservation across consecutive requests;
- platform event publication, pending-wake replay, wrapping deadlines, and
  trace call ordering; and
- the same transport fixtures with trace enabled, trace compiled out, and an
  exact C++11 compile mode.

Run all three native gates:

```sh
PLATFORMIO_CORE_DIR=/home/dave/.platformio_core_portable pio test -e native_trace_on
PLATFORMIO_CORE_DIR=/home/dave/.platformio_core_portable pio test -e native_trace_off
PLATFORMIO_CORE_DIR=/home/dave/.platformio_core_portable pio test -e native_cxx11
```

`test/support/ModbusADU.*` is an unmodified test-only fixture of the immutable
dependency revision. Production builds resolve ModbusADU from `library.json`.

These gates intentionally detect logical waits and platform-operation growth,
but they are not cycle benchmarks. GIGA/mbed object size, UART drain behavior,
thread scheduling, transceiver turnaround, and electrical bus idle remain
embedded compile and hardware gates.

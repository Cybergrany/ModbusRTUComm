# Compatibility transport tests

The native suite is a deterministic characterization of the transport replay
from the immutable source tuple recorded in `ogm-fork-lock.json`. It uses a
fake wrapping clock and timestamped `Stream`, not wall-clock sleeps.

Coverage includes:

- exact FC03 and FC69 transmitted bytes and CRC order;
- successful writes without a transceiver-local echo;
- receive-safe DE/RE initialization, then DE-high, write, drain, post-delay,
  and DE-low ordering for complete, partial, and drain-failed writes; RE is not
  toggled per transmission;
- T1.5/T3.5 boundaries, maximum legal FC03 response timing, and one-frame TX
  operation budget;
- first-byte timeout, late grace, duplicate/stray recovery, CRC/frame error
  precedence, buffer cleanup, and micros rollover;
- one-shot pre/post zero/maximum/consumption behavior on success and failure;
- no-response/broadcast gates and preservation across consecutive requests;
- RX-ring capacity, overflow counter, drop, FIFO order, index wrap, and profile
  footprint assertions;
- platform event publication, pending-wake replay, wrapping deadlines, and
  trace call ordering; and
- the same transport fixtures with trace enabled, trace compiled out, and an
  exact C++11 compile mode; and
- a metrics/diagnostics/trace-off production profile that compiles Comm as its
  own source translation unit against the exact pinned ModbusADU package.

Run all native gates:

```sh
export PLATFORMIO_CORE_DIR=/absolute/path/to/a/writable/platformio-core
pio test -e native_trace_on
pio test -e native_trace_off
pio test -e native_cxx11
pio test -e native_failure_backend
pio test -e production_profile
```

`test/support/ModbusADU.*` is an unmodified test-only fixture of the immutable
dependency revision used only by source-included characterization. The
production profile rejects that fixture and resolves the exact Git dependency
declared in `platformio.ini`/`library.json`.

For the GIGA include-provenance gate, put `test/include_shadow` first on the
include path while compiling `examples/Basic/Basic.ino`. The poison headers
make the build fail if either internal GIGA include regresses from its
package-relative form. This checks include ownership; the consumer must still
remove/exclude any second global `GigaBufferedSerial` definition to avoid ODR
violation.

These gates intentionally detect logical waits and platform-operation growth,
but they are not cycle benchmarks. GIGA/mbed object size, UART drain behavior,
thread scheduling, transceiver turnaround, and electrical bus idle remain
embedded compile and hardware gates.

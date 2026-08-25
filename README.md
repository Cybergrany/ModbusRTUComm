# ModbusRTUComm

This repository is a provenance-preserving fork of
[`CMB27/ModbusRTUComm`](https://github.com/CMB27/ModbusRTUComm). It provides the
RTU transport used by the OpenGameMaster compatibility line: frame send and
receive mechanics, timing boundaries, response classification, recovery, and
optional static platform backends around a caller-owned Arduino `Stream`.

> [!IMPORTANT]
> The functional replay is software-gated but not yet a release. Do not pin an
> OGM consumer to this branch until the exact Master/Comm tuple has passed its
> consumer builds and physical RS485 checkpoint. `main` remains the unmodified
> current CMB27 line; compatibility work exists only on `ogm/compat` and
> reviewed branches based on it.

## Repository lines

| Ref | Purpose |
| --- | --- |
| `main` | Mirrors current `CMB27/ModbusRTUComm` without OGM changes. |
| `ogm/compat` | Historical CMB27 import lineage plus reviewed OGM compatibility replays. |

Never merge or rebase `main` into `ogm/compat` as part of packaging. Newer
upstream reconciliation is a separate behavior migration with its own tests.
Exact branch points, source hashes, replay commits, and gate status are in
[`OGM_FORK_PROVENANCE.md`](OGM_FORK_PROVENANCE.md) and
[`ogm-fork-lock.json`](ogm-fork-lock.json).

## Responsibilities

`ModbusRTUComm` owns:

- T1.5/T3.5 timing and response-start, maximum-frame, late-grace, and drain
  boundaries;
- exact ADU transmission after CRC update;
- DE-high, stream write, TX drain, post-delay, and DE-low ordering;
- continuous RX ingress, frame extraction, CRC/frame/timeout classification,
  duplicate/stray/late recovery, and output-buffer cleanup;
- no-response/broadcast turnaround gates; and
- optional compile-time metrics, trace, and RX event-task support.

It does not build Modbus requests, interpret register semantics, own the
`Stream`, configure its baud/format, retry application operations, or define
any board/game/topology concepts.

## Basic use

Configure the UART before the transport. Negative direction pins mean that the
pin is not supplied.

```cpp
#include <Arduino.h>
#include <ModbusADU.h>
#include <ModbusRTUComm.h>

constexpr unsigned long kBaud = 250000UL;
ModbusRTUComm transport(Serial1, 2, 3);  // Stream, DE, optional RE

void setup() {
  Serial1.begin(kBaud, SERIAL_8N1);
  transport.begin(kBaud, SERIAL_8N1);
  transport.setTimeout(20UL);            // response-start timeout, ms
}

ModbusRTUCommError transact(ModbusADU& requestAndResponse) {
  if (!transport.writeAdu(requestAndResponse)) {
    return MODBUS_RTU_COMM_FRAME_ERROR;
  }
  return transport.readAdu(requestAndResponse);
}
```

`writeAdu()` updates the CRC and returns false for a partial write, failed drain,
or failed pre-TX cleanup. The validated transport does not require a
transceiver-local echo. `readAdu()` uses the unit/function already present in
the ADU as its expected response identity, clears the output length before
reading, and returns one of:

| Value | Meaning |
| --- | --- |
| `MODBUS_RTU_COMM_SUCCESS` (`0`) | A matching CRC-valid response was committed to the ADU. |
| `MODBUS_RTU_COMM_TIMEOUT` (`1`) | No terminal framing/CRC error outranked the response timeout. |
| `MODBUS_RTU_COMM_FRAME_ERROR` (`2`) | A terminal framing/drain error occurred. Frame damage outranks CRC damage when both are observed during resynchronization. |
| `MODBUS_RTU_COMM_CRC_ERROR` (`3`) | CRC damage was the terminal classified error. |

## One-shot scheduling gaps

Higher-level libraries may request a minimum pre- or post-transmit holdoff for
the next transaction without changing the base RTU timing:

```cpp
transport.setPreTxGapUsOnce(5000UL);
transport.setPostTxGapUsOnce(3000UL);
transport.writeAdu(request);
```

The capability macros `MBUS_RTU_COMM_COMPAT_API_VERSION` and
`MBUS_RTU_COMM_HAS_ONE_SHOT_GAPS` let a higher-level fork remain buildable
against the historical seed while selecting these APIs only when available.

## Platform binding

The default backend is
[`ArduinoModbusRTUPlatform`](src/platform/arduino/ArduinoModbusRTUPlatform.h).
The neutral contract and full backend authoring example are documented in
[`src/platform/README.md`](src/platform/README.md); GIGA/mbed behavior is in the
[`Arduino backend README`](src/platform/arduino/README.md).

A consumer can select another static backend for the whole build:

```ini
build_flags =
  -DMBUS_RTU_PLATFORM_HEADER=\"my/ModbusPlatform.h\"
  -DMBUS_RTU_PLATFORM_TYPE=my::ModbusPlatform
```

Selection is compile-time only. The facade adds no virtual dispatch, function
table, stored platform pointer, or per-call type erasure.

## Optional diagnostics and metrics

- Set `MBUS_RTU_ALLOW_DIRECT_SERIAL_DIAGNOSTICS=0` when the default console
  carries framed traffic. A custom policy can instead be selected with
  `MBUS_RTU_DIAGNOSTICS_POLICY_HEADER`.
- Set `MBUS_DETAILED_METRICS=1` for `DebugInfo` and `debugInfo()`. The header
  then defines `MBUS_RTU_COMM_HAS_DEBUG_INFO=1`.
- Set `MBUS_RTU_PLATFORM_TRACE=1` only for characterization. Trace adds clock
  calls and can perturb timing.
- RX event-task support is controlled by `MBUS_RTU_ENABLE_RX_THREAD` and
  `MBUS_RTU_RX_THREAD_STACK_BYTES`. Stack-watermark diagnostics are separately
  gated and expose `MBUS_RTU_COMM_HAS_RX_STACK_SNAPSHOT` only when active.

Every macro affecting class layout or compiled behavior must be consistent in
all translation units that include the library.

## Tests and compile gates

Run the deterministic native variants:

```sh
PLATFORMIO_CORE_DIR=/home/dave/.platformio_core_portable pio test -e native_trace_on
PLATFORMIO_CORE_DIR=/home/dave/.platformio_core_portable pio test -e native_trace_off
PLATFORMIO_CORE_DIR=/home/dave/.platformio_core_portable pio test -e native_cxx11
```

The suite freezes exact FC03/FC69 bytes and CRCs, no-local-echo behavior,
T1.5/T3.5 edges, maximum response duration, state/error precedence, buffer
cleanup, no-response gates, wraparound, DE/write/drain/delay order, task/wake
order, trace-on/off behavior, and a deterministic hot-path operation budget.
See [`test/README.md`](test/README.md).

An embedded release candidate must additionally compile the included
[`Basic`](examples/Basic/Basic.ino) example for each supported toolchain and
run the exact consumer dependency tree. Native passing results do not prove
UART interrupt latency, drain duration, mbed scheduling, RS485 electrical
turnaround, or physical compatibility with deployed firmware.

## Installing a validated compatibility release

Pin an immutable compatibility tag or full commit, never a moving branch:

```ini
lib_deps =
  https://github.com/Cybergrany/ModbusRTUComm.git#<validated-tag-or-40-char-commit>
```

`library.json` pins the reviewed ModbusADU dependency. Do not independently
override that pair without repeating the compatibility gates.

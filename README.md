# ModbusRTUComm

> [!IMPORTANT]
> This branch is the OpenGameMaster compatibility seed, not the current CMB27
> development line. It is rooted at the exact CMB27 revision originally
> imported by OGM and intentionally contains no replayed OGM transport changes
> yet. Do not use it as an OGM replacement until a validated compatibility
> release is tagged.

This library provides some core functions for implementing Modbus RTU communication.
It is not a full implementation of Modbus RTU. Other libraries are available for that purpose.

It owns RTU frame send/receive mechanics around a caller-owned Arduino
`Stream`: inter-character and inter-frame timing, response timeout, CRC/frame
classification, and optional driver/receiver direction pins. Protocol request
construction and response semantics belong in a master or slave library.

## Choose the correct line

| Ref | Purpose | Consumer guidance |
| --- | --- | --- |
| `main` | Mirrors the current `CMB27/ModbusRTUComm` line without OGM changes. | Use to review or incorporate upstream work, not as an automatic OGM upgrade. |
| `ogm/compat` | Starts at CMB27 `fb24ae3782ef8da6ad8f38f9f9eff9956edb23fc`, the source imported by OGM. | OGM changes are replayed here in small, test-gated commits. Consume only an immutable validated tag or commit. |

The exact anchors, source hashes and replay policy are recorded in
[OGM_FORK_PROVENANCE.md](OGM_FORK_PROVENANCE.md) and
[`ogm-fork-lock.json`](ogm-fork-lock.json). Do not merge or rebase `main` into
`ogm/compat`: newer-upstream reconciliation is a separate behavior change.

## Compatibility-line status

The seed keeps `src/ModbusRTUComm.h` and `src/ModbusRTUComm.cpp` unchanged from
the historical branch point. Package/provenance files have been added, but the
functional OGM replay has not begun. Check `ogm_functional_replay` in
`ogm-fork-lock.json` before treating a revision as a migration candidate.

In particular, the seed still directly uses Arduino timing, GPIO and `Stream`
calls. Platform-neutral clock, drain, direction-control, lock and diagnostic
extension points will be replayed behind the library boundary; they are not
claimed by this seed.

## Installing a validated compatibility release

PlatformIO consumers should pin an immutable compatibility tag or full commit,
never a moving branch:

```ini
lib_deps =
  https://github.com/Cybergrany/ModbusRTUComm.git#<validated-tag-or-40-char-commit>
```

The pinned `library.json` also pins the reviewed CMB27 `ModbusADU` revision.
Do not independently override that dependency unless the pair is revalidated.
For local extraction work, a path dependency keeps the consumer and library
changes in one test run:

```ini
lib_deps =
  symlink:///absolute/path/to/ModbusRTUComm
```

Before publishing a release, replace the local path with the immutable remote
revision and rebuild the exact OGM dependency tree from a clean dependency
cache.

## Compatibility-seed usage example

The caller configures the serial port first. On this historical seed,
`ModbusRTUComm::begin()` derives RTU timing and initializes optional direction
pins; it does not call `HardwareSerial::begin()`.

```cpp
#include <Arduino.h>
#include <ModbusADU.h>
#include <ModbusRTUComm.h>

namespace {
constexpr unsigned long kBaud = 115200UL;
constexpr int8_t kDriverEnablePin = 2;
constexpr int8_t kReceiverEnablePin = 3;
ModbusRTUComm transport(Serial1, kDriverEnablePin, kReceiverEnablePin);
}

void setup() {
  Serial1.begin(kBaud, SERIAL_8N1); // configure the UART first
  transport.begin(kBaud, SERIAL_8N1);
  transport.setTimeout(500UL);
}

ModbusRTUCommError receiveOne(ModbusADU& response) {
  return transport.readAdu(response);
}

bool sendOne(ModbusADU& request) {
  // writeAdu() updates the CRC before writing and returns whether the seed's
  // historical local-echo verification path succeeded.
  return transport.writeAdu(request);
}
```

The application or higher-level library must populate and validate the
`ModbusADU`. `readAdu()` returns one of `MODBUS_RTU_COMM_SUCCESS`,
`MODBUS_RTU_COMM_TIMEOUT`, `MODBUS_RTU_COMM_FRAME_ERROR` or
`MODBUS_RTU_COMM_CRC_ERROR`. Consult the header at the pinned release for the
exact API after the functional replay.

## OGM compatibility contract

Moving transport ownership into this repository must not change:

- exact transmitted bytes, CRC placement or received-frame boundaries;
- inter-character/inter-frame and response-timeout boundary behavior;
- serial write, drain, post-delay and DE/RE transition order;
- receive-buffer handling, local-echo behavior and error classification;
- no-response/broadcast call ordering expected by higher-level libraries;
- stack/RAM/flash footprint and hot-path performance beyond accepted gates.

Native fakes can verify call order, timeout arithmetic and byte sequences, but
they cannot prove UART interrupt latency, RS485 electrical timing, scheduler
behavior, drain duration or bus contention. No Stage C bridge or hardware-
validation claim is made by this seed documentation.

# OpenGameMaster fork provenance

This repository retains the complete CMB27 Git history. Its two long-lived
lines have deliberately different purposes:

- `main` tracks `upstream/main` without OGM changes.
- `ogm/compat` starts at the historical source revision imported by OGM and is
  the only line on which reviewed OGM compatibility changes may be replayed.

Do not recreate `ogm/compat` from the latest upstream API. Doing so would mix
the later upstream redesign with the repository move and invalidate the
no-behaviour-change migration gate.

## Immutable anchors

| Purpose | Commit |
| --- | --- |
| CMB27 revision reviewed before fork seeding | `d71a38ea572570e3877fe21cf29313f7312bc772` |
| `ogm/compat` branch point | `fb24ae3782ef8da6ad8f38f9f9eff9956edb23fc` |
| First OGM import in `OpenGameMaster_pio` | `ef4d0ccb1cccd60781f3605d444a16dc7900661c` |
| Later transfer into `OGM_Portable` | `a3551eab76877052df39a5e3ac875f71ce4e1109` |

The two files imported by `ef4d0ccb` were compared with CMB27 at the branch
point. Their SHA-256 values match exactly:

| Source | SHA-256 |
| --- | --- |
| `src/ModbusRTUComm.h` | `e78548cc7ffa72dfab5f82335540656647e7135acb80b7d809b80790bd41dbbd` |
| `src/ModbusRTUComm.cpp` | `bc5049d415727da1c4d51b6850423a317824e585b83a3cc51f4f47b4efe64bcd` |

The machine-readable form of these anchors and the package dependency pin is
[`ogm-fork-lock.json`](ogm-fork-lock.json).

## Current compatibility status

The current replay candidate moves the platform boundary and complete transport
state machine from immutable OGM_Portable commit
`2055adb449c1e767217f09f99efda32e52a0306d` into this fork. It contains no OGM
pin, board, game, child, or topology dependency. The package boundary changes
only include paths, neutral capability macros, diagnostics-policy naming, and a
GIGA serial-format namespace; transport control flow remains the source anchor.

| Replay theme | Fork commit | Source anchor/themes |
| --- | --- | --- |
| Static platform facade and Arduino/mbed backend | `2e5e6971ed41e65860676a3532b5f4f7b5781994` | `2055adb`; principally `2cb91ce`, `c95cdf5`, `cfbd946`, `e0cd769`, `59403e1` |
| RTU RX/TX state machine and characterization suite | `ab792d140609e50e4778f664ab87895ccffc4abb` | `2055adb`; snapshot `a3551eab` plus reviewed fixes through `0bb8ed6` |

The raw OGM source files at that anchor were captured before package-only
adjustments:

| OGM source | SHA-256 |
| --- | --- |
| `include/IO/comms/MasterComms/ModbusRTUComm.h` | `8351f696024800071fad5e8346c670e7a97626567a2bfa984f7f2dafffe99273` |
| `include/IO/comms/MasterComms/ModbusRTUComm.cpp` | `cdcaf65dee84e0dea30442e70fca26bcc75b298d1e274d7aa10b67e282226c0d` |
| `include/IO/comms/ModbusRTUPlatform.h` | `0149753b168d5f2146c9f1630476585c05f308c3e9d99580407b0d7c0b08dcde` |
| `include/platform/arduino/ArduinoModbusRTUPlatform.h` | `2b6ee67d29f8ef6a82da0ab0aa2ead62e7e3b739644c64f99b4005f7f65c4ded` |

This is a software candidate, not a compatibility release. Native and
toolchain gates are recorded below, but the paired Master consumer build and
physical GIGA/RS485 checkpoint remain pending.

## Candidate software evidence

- Trace-enabled native characterization: `34/34` passed.
- Trace-compiled-out characterization: `30/30` passed.
- Exact C++11, pedantic warnings-as-errors characterization: `30/30` passed.
- Arduino GIGA/mbed compile with RX event task enabled: passed.
- Arduino Nano/AVR compile: passed; the deliberately complete example consumes
  `3107/2048` bytes of Nano RAM and is therefore a compile gate, not a deployable
  Nano footprint claim.
- Deterministic fixtures lock exact FC03/FC69 bytes and CRCs, current
  no-local-echo behavior, T1.5/T3.5 edges, maximum-frame timing, terminal error
  precedence, ADU cleanup, no-response gates, micros rollover, RX state order,
  DE/write/drain/delay order, partial-write cleanup, event publication/wake
  order, and a single-frame-wait TX operation budget.

The GIGA example reports `100688` bytes flash and `51168` bytes RAM, but this is
only a standalone compile. Paired whole-firmware map/resource comparison must
use the exact Master dependency tuple before advancing to hardware.

## Remote layout

```text
origin    git@github.com:Cybergrany/ModbusRTUComm.git
upstream  https://github.com/CMB27/ModbusRTUComm.git
```

The Cybergrany repository is maintained as a GitHub fork of CMB27. Refresh the
upstream tracking line and publish compatibility work without rewriting either
history:

```bash
git fetch upstream --prune --tags
git push origin upstream/main:main
git push -u origin ogm/compat
```

The `upstream/main:main` push must be fast-forward-only in practice. If Git
rejects it, inspect the fork divergence; do not force-push either long-lived
line.

## Replay and release policy

1. Replay OGM transport work as small, reviewable themes and record the source
   OGM commits in each commit message.
2. Keep platform implementations behind the neutral platform contract; no OGM
   pin, board, game, or topology headers belong in the public library API.
3. Preserve the existing framing, timeout, no-response, byte-order, and
   performance oracles before changing consumers.
4. Pin releases by an immutable tag or commit. Do not consume `main` or
   `ogm/compat` by a moving branch name.
5. Reconcile newer CMB27 changes only in a separate, explicitly test-gated
   change after the compatibility package is proven.

## Compatibility release gates

A commit or tag on `ogm/compat` is suitable for an OGM consumer only after all
of the following evidence is recorded against the exact dependency tuple:

1. Source provenance: each replay commit names its source OGM commit(s), and
   package locks resolve to immutable dependency commits.
2. Frame parity: transmitted bytes, CRCs, receive boundaries, buffer cleanup,
   local echo and timeout/frame/CRC errors match frozen OGM fixtures.
3. Ordering and timing parity: serial write, drain, delays, DE/RE transitions,
   locks and no-response completion occur in the established order.
4. Resource/performance parity: paired timing gates pass and embedded
   stack/RAM/flash changes are understood and accepted.
5. Consumer validation: native suites and exact supported OGM master, bridge
   and legacy-slave firmware builds pass from clean dependency caches.
6. Hardware validation: the release candidate is exercised over the physical
   GIGA/RS485 topology with unchanged deployed slave firmware.

Passing items 1-5 supports a hardware checkpoint because the migration should
not alter on-wire behavior; it does not replace item 6. Hardware that merely
appears playable likewise does not replace the trace, ordering or performance
gates.

The compatibility manifest pins ModbusADU to
`7cb0e24f0abe86bc83e114325d75fe7a7d878562`, whose implementation is the one
originally imported by OGM and remains the reviewed upstream revision.

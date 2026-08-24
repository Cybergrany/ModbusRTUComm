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

The initial `ogm/compat` seed adds provenance and package metadata only. Its
transport source files remain identical to the historical CMB27 branch point,
and `ogm_functional_replay` in `ogm-fork-lock.json` remains `not_started` until
the first reviewed source replay lands. A successful seed compile demonstrates
that the historical package can still be resolved; it does not demonstrate
that current OGM consumers can switch to it without a behavior delta.

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

The seed manifest pins ModbusADU to
`7cb0e24f0abe86bc83e114325d75fe7a7d878562`, whose implementation is the one
originally imported by OGM and remains the reviewed upstream revision.

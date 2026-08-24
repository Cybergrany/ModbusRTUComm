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

## Remote layout

```text
origin    git@github.com:Cybergrany/ModbusRTUComm.git
upstream  https://github.com/CMB27/ModbusRTUComm.git
```

The Cybergrany repository must be created as a GitHub fork of CMB27 before the
local compatibility branch can be pushed. Once it exists:

```bash
git fetch upstream --prune --tags
git push -u origin ogm/compat
```

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

The seed manifest pins ModbusADU to
`7cb0e24f0abe86bc83e114325d75fe7a7d878562`, whose implementation is the one
originally imported by OGM and remains the reviewed upstream revision.

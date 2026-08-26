# Basic example

`Basic.ino` uses `Serial` for Modbus so it refuses to compile while direct
serial diagnostics are enabled. Set the policy for every translation unit:

```ini
build_flags =
  -DMBUS_RTU_ALLOW_DIRECT_SERIAL_DIAGNOSTICS=0
```

Defining the macro only above an include in the sketch is insufficient because
`ModbusRTUComm.cpp` is compiled separately. If a dedicated bus such as
`Serial1` is used and the console is electrically/logically separate, adapt the
example and choose the diagnostics policy deliberately.

PlatformIO resolves the declared ModbusADU dependency from this repository's
`library.json`. Arduino Library Manager metadata cannot express a Git commit,
so Arduino IDE users should install a compatible ModbusADU revision explicitly.

# KIA K7 YG HEV Panda Port

[← Documentation index](../README.md)

## Runtime

- `pandad` owns Panda USB through `libusb` and publishes CAN batches to
  the ordered `/dev/shm/edgepilot_can` shared-memory ring queue.
- `controlsd` runs the standalone K7 controller at 100 Hz and publishes
  generated CAN batches to the ordered `/dev/shm/edgepilot_sendcan` ring queue.
- `pandad` is the final TX gate. `EDGEPILOT_PANDA_TX=0` is the default and
  prevents every generated frame from reaching USB.
- No openpilot checkout or Python DBC extension is required on the board.

The CAN queues have 64 slots, reject new batches instead of overwriting older
ones when full, and are drained in sequence order. Producer startup resets its
own queue generation, and `pandad` drops TX batches older than 100 ms.
The one-second daemon logs expose queue depth, full/stale counts, Panda CAN
errors, blocked frames, heartbeat status, USB retries, and malformed RX batches.

The controller uses the validated K7 YG HEV bus split:

- RX bus 0: powertrain, cluster, SAS, brake, gear, and body state.
- RX bus 1: MDPS state.
- RX bus 2: camera `LKAS11` seed.
- TX bus 0: `LKAS11` at 100 Hz.
- TX bus 1: mirrored `LKAS11` at 100 Hz and `CLU11` at 50 Hz.
- TX bus 2: `MDPS12` at 100 Hz.

While steering is active below the MDPS threshold, the bus-1 `CLU11` helper
reports 60 kph (38 mph) and preserves the source decimal-speed field. This
matches the K7 branch in the reference openpilot controller.

Runtime parameters live in `params/`; see [params/README.md](../params/README.md).

## MaixCAM2 connection

The MaixCAM2 has a single USB-C port, which carries the Panda in host mode, so
the board needs power from another source; this wiring is not finished yet.
Until it is, the manager starts `pandad` only with `EDGEPILOT_ENABLE_PANDA=1`.
With it set, the manager switches the port to host
(`/sys/class/usb_role/8000000.dwc3-role-switch/role`) before starting the
processes and restores the previous role on exit.

## Build

Build and upload as in [Build and deploy](build-and-deploy.md). The build
container installs `libusb-1.0-0-dev`; the board needs the `libusb-1.0` runtime
library for `pandad`.

## Offline Validation

Export one 60 s chunk of a continuous drive (a K230 recording, until the recorder
is ported) to a `K230CAN1` fixture and replay it through the controller (see
[gtest/README.md](../gtest/README.md) for why a parked chunk fails):

```sh
python3 tools/control/export_can_fixture.py <route>/events/003.bin drive.can
./build-host/bin/gtest_control_replay drive.can
```

The 60.001 second K7 YG HEV fixture contains 43,273 CAN records. The expected
result is 5,970 messages each for bus-0 `LKAS11`, bus-1 `LKAS11`, and bus-2
`MDPS12`, plus 2,985 bus-1 `CLU11` messages. The replay also checks frame
lengths, active ticks, torque bounds, and the desired curvature against the
openpilot reference.

## Shadow Run

```sh
EDGEPILOT_ENABLE_PANDA=1 \
EDGEPILOT_ENABLE_CONTROL=1 \
EDGEPILOT_PANDA_TX=0 \
EDGEPILOT_PANDA_SAFETY=nooutput \
python3 /root/edgepilot/manager.py
```

Use `hyundaiCommunity` only after the connected vehicle fingerprint and Panda
health confirm the expected K7 configuration. Collected logs from the current
vehicle report `mdpsBus=1`, `sasBus=1`, and `hyundaiCommunity:0`.

## TX Gates

Vehicle transmission requires every explicit setting below:

```sh
EDGEPILOT_ENABLE_PANDA=1
EDGEPILOT_ENABLE_CONTROL=1
EDGEPILOT_PANDA_TX=1
EDGEPILOT_PANDA_SAFETY=hyundaiCommunity
EDGEPILOT_PANDA_ENGAGED=1
```

Keep `EDGEPILOT_FORCE_ENGAGED=0` in a vehicle. Engagement must come from the
vehicle SET/CANCEL button state. Before any closed-course TX test, verify Panda
USB RX, ignition, safety mode/param, `controls_allowed`, CAN freshness, checksum
counters, and zero blocked/error counts in shadow mode.

# Betaflight SITL + Gazebo

Fly the `atalef_betaflight` model in Gazebo with a Betaflight SITL flight
controller, drive it from a virtual RC receiver, and hand control to your
companion computer over MSP using **MSP OVERRIDE**.

## Data flow

```
                 fdm_packet  (state, UDP 9003)
   Gazebo  ─────────────────────────────────────▶  Betaflight SITL
 (BetaFlightPlugin) ◀────────────────────────────  (betaflight_SITL.elf)
                 servo_packet (motors, UDP 9002)

   virtual_rc.py ──── rc_packet (UDP 9004) ───────▶  Betaflight SITL   (the "receiver")

   companion C++ app ◀── MSP telemetry (TCP 5761) ─▶  Betaflight SITL
                     ─── MSP_SET_RAW_RC (TCP 5761) ─▶  (applied to masked
                                                        channels while
                                                        MSP OVERRIDE is active)
```

## 1. Run Gazebo

```bash
colcon build --packages-select ardupilot_gazebo
source install/setup.bash
gz sim -v4 -r kfar_netter_betaflight.sdf
```

The `BetaFlightPlugin` binds UDP **9002** (motors in) and sends state to UDP
**9003**. Ports are configurable in the world's `<experimental:params>`.

## 2. Run Betaflight SITL

From the betaflight checkout:

```bash
make TARGET=SITL
./obj/main/betaflight_SITL.elf 127.0.0.1
```

`MSP OVERRIDE` (box id 50) is compiled into the SITL target by default — no
source changes needed.

## 3. Configure the FC (one-time, over CLI on tcp://127.0.0.1:5761)

Connect with the Betaflight Configurator or any MSP/CLI client and paste:

```
# --- mode switches (aux <index> <mode-permanent-id> <aux-ch 0=AUX1> <start> <end> <logic>) ---
aux 0 0  0 1700 2100 0 0      # ARM          on AUX1 high
aux 1 1  1 1700 2100 0 0      # ANGLE        on AUX2 high
aux 2 50 2 1700 2100 0 0      # MSP OVERRIDE on AUX3 high

# --- which channels the companion app may drive while MSP OVERRIDE is active ---
# bitmask: bit0=Roll bit1=Pitch bit2=Throttle bit3=Yaw bit4=AUX1 ...
# 15 = Roll+Pitch+Throttle+Yaw (sticks only; keep arm on the virtual RC).
set msp_override_channels_mask = 15

# optional: allow MSP-only control even if the RC link is "lost"
# set msp_override_failsafe = ON

save
```

Notes:
* `<mode>` in the `aux` command is the box **permanent id** (ARM=0, ANGLE=1,
  MSP OVERRIDE=50), not the row index.
* The AUX channel used to *trigger* MSP OVERRIDE (AUX3 here) is automatically
  excluded from the override mask, so the switch always stays under RC control.
* To let the app also arm, add AUX1 to the mask (`15 + 16 = 31`).

## 4. Run the virtual RC receiver

```bash
./tools/virtual_rc.py                 # defaults: 127.0.0.1:9004, arm=AUX1, angle=AUX2, msp=AUX3
./tools/virtual_rc.py --msp-aux 4     # move the MSP OVERRIDE switch to AUX4
```

Keys (OS key-repeat holds a stick deflected):

| key | action |
|-----|--------|
| `w` / `s` | throttle up / down (latched) |
| `a` / `d` | yaw left / right |
| `i` / `k` | pitch forward / back |
| `j` / `l` | roll left / right |
| `SPACE`   | arm / disarm toggle (AUX1) |
| `g`       | angle mode toggle (AUX2) |
| `m`       | **MSP OVERRIDE** toggle (AUX3) |
| `x`       | panic: cut throttle + disarm |
| `c`       | re-centre sticks |
| `q`       | quit (disarms) |

### Gamepad mode

Flying with the keyboard is fine for a quick check, but a gamepad is far nicer.
`virtual_rc.py` reads the Linux joystick API (`/dev/input/jsN`) directly — no
extra Python packages required.

```bash
# 1) discover your controller's axis/button numbers:
./tools/virtual_rc.py --joystick --list        # move sticks, press buttons, Ctrl-C

# 2) fly (Xbox-style defaults shown; remap if --list showed different numbers):
./tools/virtual_rc.py --joystick               # /dev/input/js0
./tools/virtual_rc.py --joystick /dev/input/js1 \
    --axes "roll=3,pitch=4i,throttle=1i,yaw=0" \
    --buttons "arm=0,panic=1,angle=2,msp=3,center=7,quit=6"
```

* `--axes name=number[i]` — a trailing `i` inverts (pitch/throttle usually need
  it so "stick up" = nose-up / more throttle). Names: `roll pitch throttle yaw`.
* `--buttons name=number` — rising-edge toggles. Names: `arm angle msp panic
  center quit`. The **`msp`** button is your MSP OVERRIDE switch (AUX3).
* Throttle defaults to `--throttle-mode incremental`: because gamepad sticks
  spring back to centre, pushing the throttle stick changes throttle at a rate
  (`--throttle-rate` µs/s) and it latches when you release — like a real
  throttle. Use `--throttle-mode direct` if you have a non-centring throttle
  axis. `--deadzone` sets the centre dead band.

### Typical session
1. Start Gazebo, then the SITL, then `virtual_rc.py` (keyboard or `--joystick`).
2. Enable angle mode (`g` / the angle button) for easy self-levelled flight.
3. Raise throttle a little, then arm (`SPACE` / the arm button). Fly.
4. Flip MSP OVERRIDE **on** (`m` / the msp button): the channels in
   `msp_override_channels_mask` now come from your companion app's
   `MSP_SET_RAW_RC` messages (TCP 5761) instead of the sticks.
5. Flip it off again to hand control back to the virtual sticks.

## Companion computer (MSP)

Your C++ app connects to `tcp://127.0.0.1:5761` (SITL UART1 = MSP). It already
reads telemetry via MSP; to take control while MSP OVERRIDE is active, send
`MSP_SET_RAW_RC` (command 200) periodically with the channel values (AETR
order) for the masked channels. Send at a steady rate (e.g. 50–100 Hz) so the
values stay fresh.

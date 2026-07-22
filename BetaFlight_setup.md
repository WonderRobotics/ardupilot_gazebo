# Betaflight SITL + Gazebo — build & run

Fly the `atalef_betaflight` model in Gazebo with a **Betaflight SITL** flight
controller, drive it with a virtual RC receiver, and hand control to a
companion app over MSP. The ArduPilot setup is untouched — this adds a parallel
`BetaFlightPlugin`, model, and world.

## Data flow / ports

```
                   fdm_packet  (state,  UDP 9003)
   Gazebo  ───────────────────────────────────────▶  Betaflight SITL
 (BetaFlightPlugin) ◀─────────────────────────────── (betaflight_SITL.elf)
                   servo_packet (motors, UDP 9002)

   virtual_rc.py ───── rc_packet (UDP 9004) ────────▶  Betaflight  (the receiver)
   companion app ◀──── MSP (TCP 5761, UART1) ───────▶  Betaflight
```

| port | direction | purpose |
|------|-----------|---------|
| 9002 | BF → Gazebo | motor outputs (`servo_packet`) |
| 9003 | Gazebo → BF | sim state (`fdm_packet`, 144 B) |
| 9004 | vrc → BF | RC channels (`rc_packet`, AETR order) |
| 5761 | TCP | MSP / CLI (UART1) for configurator & companion app |

## 1. Build the Gazebo plugin

The workspace is an ament/colcon package, so it needs **ROS 2 (Jazzy) and the
system Python** (which has `catkin_pkg`).
If a virtualenv is active it will use
the wrong Python and `ament_package()` fails — so deactivate it first.

```bash
deactivate 2>/dev/null  # leave any active virtualenv
source /opt/ros/jazzy/setup.bash
cd /home/matan/gz_ws2/src
colcon build --packages-select ardupilot_gazebo
```

> The possible `CMake … CMP0146` / OpenCV warning is harmless (it comes from
> `GstCameraPlugin`'s OpenCV dependency, not this plugin).

## 2. Build Betaflight SITL (once)

```bash
cd <your-betaflight-repo>
make TARGET=SITL # produces obj/main/betaflight_SITL.elf
```

`MSP OVERRIDE` (box id 50) is compiled in by default — no source changes needed.

## 3. Run

```bash
# Terminal 1 — Gazebo
cd ~/gz_ws/src/ardupilot_gazebo
source install/setup.bash
gz sim -v4 -r kfar_netter_betaflight.sdf
```
Make sure the sim is actually **running** (GUI bottom bar: RTF > 0, Sim Time
increasing). If it's paused, press ▶ — the plugin only streams state while
stepping.

```bash
# Terminal 2 — Betaflight SITL
cd <your-betaflight-repo>
./obj/main/betaflight_SITL.elf 127.0.0.1
```
On success it prints `[SITL] new fdm 144 t:… ` (state link up) shortly after start.

```bash
# Terminal 3 — Virtual RC
cd ~/gz_ws/src/ardupilot_gazebo
./tools/custom_virtual_rc.py                 # keyboard
./tools/custom_virtual_rc.py --joystick      # gamepad (run --list first to map it)
```
Keep this running the whole time — if RC stops streaming, the FC goes to failsafe.

## 4. Configure the FC (once — persists to eeprom.bin)

You can connect using the Betaflight configurator.
First use
`websockify 127.0.0.1:6761 127.0.0.1:5761` and then open in the browser BF configurator, change port to `ws://127.0.0.1:6761` and connect. Then map all required channels and test with the virtual RC.

**Alternative**:

Talk to the CLI over TCP 5761. Enter the CLI by
sending `#`, and **always leave with `exit`** (staying in the CLI blocks arming
and freezes the RX).

```bash
socat -,raw,echo=0 tcp:127.0.0.1:5761
```
```bash
aux 0 0  0 1700 2100 0 0      # ARM          on AUX1 high
aux 1 1  1 1700 2100 0 0      # ANGLE        on AUX2 high
aux 2 50 2 1700 2100 0 0      # MSP OVERRIDE on AUX3 high
set msp_override_channels_mask = 15    # Roll+Pitch+Throttle+Yaw
set max_aux_channels = 14     # SITL by default only 6 so not enought for our usual setup
map                                    # verify AETR1234
save
```
`<mode>` is the box **permanent id** (ARM=0, ANGLE=1, MSP OVERRIDE=50);
`<aux>` is 0-based (AUX1=0).

Alternatively, 

## 5. Fly

1. In the vrc: `g` = angle (self-level) mode, throttle at minimum.
2. `SPACE` to arm — the props should idle-spin.
3. Push throttle up (~1500+) to lift off. Keyboard is slow; hold `w`, raise
   `--throttle-step`, or use `--joystick`.
4. `x` = panic (cut throttle + disarm).

Controls and gamepad mapping are documented in [tools/README.md](tools/README.md).

## 6. MSP OVERRIDE — control from a companion app

While the `m` switch (AUX3) is **on**, the channels in
`msp_override_channels_mask` are taken from `MSP_SET_RAW_RC` messages your app
sends over MSP instead of the sticks. Flip it off to return to the vrc.

The SITL's MSP is **TCP 5761**. Our serial-based library 
[christianrauch/msp](https://github.com/christianrauch/msp) needs a virtual
serial device — so we bridge it with socat in **new terminal**:

```bash
socat PTY,link=$HOME/bf_vcom,raw,echo=0 TCP:127.0.0.1:5761   # baud is ignored over the PTY
```
Then from another terminal, link it to e.g. `/dev/ttyUSB0` (and set permissions if needed):
```bash
sudo ln -sf $HOME/bf_vcom /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0 # This line may not be needed
```
Finally, we can connect in DS using this serial port.

**Note**: only one client at a time can use 5761.

## Troubleshooting

| symptom | cause / fix |
|---------|-------------|
| BF never prints `new fdm` | sim is paused — press ▶ (RTF > 0). The plugin only sends while stepping. |
| `Arming disable flags: RXLOSS CLI` | you're in the CLI — send `exit`. The CLI freezes RX and blocks arming. |
| Won't arm | check flags live in the Configurator top bar; keep the vrc streaming; keep throttle low until armed; toggle the arm switch off→on if `ARM_SWITCH` is set. |
| Armed but no lift-off | throttle too low — push to ~1500+. |
| MSP-override does nothing / throttle dies | the `m` switch is on but the app isn't sending `MSP_SET_RAW_RC` — keep it off for manual flying. |
| `colcon build` fails: `No module named 'catkin_pkg'` | a venv is active — deactivate, or force `-DPYTHON_EXECUTABLE=/usr/bin/python3`. |

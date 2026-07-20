#!/usr/bin/env python3
"""Custom virtual RC receiver for Betaflight SITL (keyboard or gamepad).

Same UDP `rc_packet` transport as `virtual_rc.py` (127.0.0.1:9004, AETR order),
with two extra features:

1. STICK MIRRORING
   The live roll / pitch / yaw / throttle values are also copied onto a block
   of AUX channels (default AUX7..AUX10), so a companion app can read the
   pilot's stick intent on dedicated channels while your flight channels stay
   AETR. Order: AUX7=roll, AUX8=pitch, AUX9=yaw, AUX10=throttle.
   Disable with `--mirror-aux 0`.
   NOTE: Betaflight only reads up to (max_aux_channels + 4) channels, and the
   SITL default is max_aux_channels = 6 (i.e. AUX1..AUX6). To make it see the
   mirror block, in the CLI run: `set max_aux_channels = 14` then `save`.

2. MULTI-STATE OVERRIDE SWITCH
   Instead of a plain on/off, the override channel (default AUX3) is a switch
   with N configurable ranges. A key cycles through them; the channel is set to
   the midpoint of the selected range. Default 4 states:
       state 0: 1000-1200   (e.g. MSP override OFF)
       state 1: 1201-1450
       state 2: 1451-1550
       state 3: 1551-2000
   On the FC, bind whatever modes you want to those sub-ranges (Modes tab or
   `aux` CLI), e.g. MSP OVERRIDE active only in 1451-2000.

Controls (keyboard; OS key-repeat holds a stick deflected):
    throttle w/s   yaw a/d   pitch i/k   roll j/l
    SPACE arm    g angle    x panic    c centre    q quit
    m  = next override state (wraps)     n = previous state
    1..9 = jump directly to override state N

Gamepad ( --joystick ): sticks fly; buttons arm / angle / panic; the `msp`
button advances the override state.
"""

import argparse
import math
import os
import select
import signal
import socket
import struct
import sys
import termios
import time
import tty

LOW = 1000
MID = 1500
HIGH = 2000
NUM_CHANNELS = 16
RC_PACKET = struct.Struct("<d16H")  # double timestamp + 16 x uint16

CH_ROLL, CH_PITCH, CH_THROTTLE, CH_YAW = 0, 1, 2, 3
AXIS_TO_CH = {"roll": CH_ROLL, "pitch": CH_PITCH,
              "throttle": CH_THROTTLE, "yaw": CH_YAW}
# Order in which the sticks are mirrored onto the AUX block.
MIRROR_SRC = [CH_ROLL, CH_PITCH, CH_YAW, CH_THROTTLE]  # roll, pitch, yaw, thrust


def clamp(v, lo=LOW, hi=HIGH):
    return max(lo, min(hi, int(v)))


def onoff(b):
    return "\033[32mON \033[0m" if b else "\033[90moff\033[0m"


def aux_to_index(aux_number):
    """AUX1 -> channel index 4, AUX2 -> 5, ..."""
    return 3 + aux_number


def apply_deadzone(v, dz):
    if abs(v) < dz:
        return 0.0
    return (v - math.copysign(dz, v)) / (1.0 - dz)


def parse_map(spec, allow_invert):
    out = {}
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        name, _, val = part.partition("=")
        name, val = name.strip(), val.strip()
        invert = False
        if allow_invert and val.endswith("i"):
            invert, val = True, val[:-1]
        out[name] = (int(val), invert)
    return out


def parse_states(spec):
    """Parse "1000-1200,1201-1450,..." into [(lo,hi), ...]."""
    states = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        lo, _, hi = part.partition("-")
        states.append((int(lo), int(hi)))
    if not states:
        raise ValueError("need at least one override range")
    return states


class Joystick:
    """Minimal reader for the Linux joystick API (/dev/input/jsN)."""

    JS_EVENT_BUTTON = 0x01
    JS_EVENT_AXIS = 0x02
    JS_EVENT_INIT = 0x80
    EVENT = struct.Struct("<IhBB")  # time, value, type, number  (8 bytes)

    def __init__(self, path):
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.axes = {}
        self.buttons = {}
        self.raw_events = []

    def poll(self):
        presses = []
        while True:
            try:
                data = os.read(self.fd, self.EVENT.size)
            except (BlockingIOError, OSError):
                break
            if not data or len(data) < self.EVENT.size:
                break
            _, value, etype, number = self.EVENT.unpack(data)
            self.raw_events.append((etype, number, value))
            base = etype & ~self.JS_EVENT_INIT
            if base == self.JS_EVENT_AXIS:
                self.axes[number] = value
            elif base == self.JS_EVENT_BUTTON:
                prev = self.buttons.get(number, 0)
                self.buttons[number] = value
                if value and not prev and not (etype & self.JS_EVENT_INIT):
                    presses.append(number)
        return presses

    def axis(self, number):
        return self.axes.get(number, 0) / 32767.0

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


class CustomVirtualRC:
    def __init__(self, args):
        self.args = args
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (args.host, args.port)

        self.arm_idx = aux_to_index(args.arm_aux)
        self.angle_idx = aux_to_index(args.angle_aux)
        self.msp_idx = aux_to_index(args.msp_aux)

        # Multi-state override switch.
        self.msp_states = parse_states(args.msp_states)
        self.msp_state = 0

        # Stick-mirror AUX block (0 disables).
        self.mirror_idx = aux_to_index(args.mirror_aux) if args.mirror_aux else 0

        self.channels = [LOW] * NUM_CHANNELS
        self.channels[CH_ROLL] = MID
        self.channels[CH_PITCH] = MID
        self.channels[CH_THROTTLE] = LOW
        self.channels[CH_YAW] = MID

        self.armed = False
        self.angle = False
        self.start = time.monotonic()
        self._status_lines = 0

        self.axis_map = parse_map(args.axes, allow_invert=True)
        button_map = parse_map(args.buttons, allow_invert=False)
        self.button_action = {num: name for name, (num, _) in button_map.items()}
        self.js = None

    # --- override state helpers -------------------------------------------
    def msp_value(self):
        lo, hi = self.msp_states[self.msp_state]
        return (lo + hi) // 2

    def set_msp_state(self, idx):
        self.msp_state = idx % len(self.msp_states)

    # --- shared actions ----------------------------------------------------
    def toggle(self, action):
        if action == "arm":
            self.armed = not self.armed
        elif action == "angle":
            self.angle = not self.angle
        elif action == "msp_next":
            self.set_msp_state(self.msp_state + 1)
        elif action == "msp_prev":
            self.set_msp_state(self.msp_state - 1)
        elif action == "panic":
            self.armed = False
            self.channels[CH_THROTTLE] = LOW
        elif action == "center":
            for idx in (CH_ROLL, CH_PITCH, CH_YAW):
                self.channels[idx] = MID

    def apply_channels(self):
        self.channels[self.arm_idx] = HIGH if self.armed else LOW
        self.channels[self.angle_idx] = HIGH if self.angle else LOW
        self.channels[self.msp_idx] = self.msp_value()
        # Mirror sticks onto the AUX block.
        if self.mirror_idx:
            for slot, src in enumerate(MIRROR_SRC):
                dst = self.mirror_idx + slot
                if dst < NUM_CHANNELS:
                    self.channels[dst] = self.channels[src]

    def send(self):
        self.apply_channels()
        t = time.monotonic() - self.start
        self.sock.sendto(RC_PACKET.pack(t, *self.channels), self.addr)

    def safe_shutdown(self):
        self.armed = False
        self.channels[CH_THROTTLE] = LOW
        for idx in (CH_ROLL, CH_PITCH, CH_YAW):
            self.channels[idx] = MID
        try:
            self.send()
        except OSError:
            pass

    # --- keyboard mode -----------------------------------------------------
    def handle_key(self, key):
        d = self.args.deflection
        step = self.args.throttle_step
        c = self.channels
        if key == "w":
            c[CH_THROTTLE] = clamp(c[CH_THROTTLE] + step)
        elif key == "s":
            c[CH_THROTTLE] = clamp(c[CH_THROTTLE] - step)
        elif key == "a":
            c[CH_YAW] = clamp(MID - d)
        elif key == "d":
            c[CH_YAW] = clamp(MID + d)
        elif key == "i":
            c[CH_PITCH] = clamp(MID + d)
        elif key == "k":
            c[CH_PITCH] = clamp(MID - d)
        elif key == "j":
            c[CH_ROLL] = clamp(MID - d)
        elif key == "l":
            c[CH_ROLL] = clamp(MID + d)
        elif key == " ":
            self.toggle("arm")
        elif key == "g":
            self.toggle("angle")
        elif key == "m":
            self.toggle("msp_next")
        elif key == "n":
            self.toggle("msp_prev")
        elif key.isdigit() and key != "0":
            self.set_msp_state(int(key) - 1)
        elif key == "x":
            self.toggle("panic")
        elif key == "c":
            self.toggle("center")
        elif key in ("q", "\x03"):
            return False
        return True

    def decay(self):
        for idx in (CH_ROLL, CH_PITCH, CH_YAW):
            v = self.channels[idx]
            if v != MID:
                self.channels[idx] = MID + int((v - MID) * 0.4)
                if abs(self.channels[idx] - MID) < 5:
                    self.channels[idx] = MID

    def run_keyboard(self):
        period = 1.0 / self.args.rate
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        running = True
        try:
            tty.setcbreak(fd)
            next_draw = 0.0
            while running:
                loop_start = time.monotonic()
                while select.select([sys.stdin], [], [], 0)[0]:
                    if not self.handle_key(sys.stdin.read(1)):
                        running = False
                        break
                self.decay()
                self.send()
                if loop_start >= next_draw:
                    self.render()
                    next_draw = loop_start + 0.05
                sleep = period - (time.monotonic() - loop_start)
                if sleep > 0:
                    time.sleep(sleep)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self.safe_shutdown()
            sys.stdout.write("\nDisarmed. Bye.\n")

    # --- gamepad mode ------------------------------------------------------
    def joystick_axes(self, dt):
        js, dz = self.js, self.args.deadzone
        for name in ("roll", "pitch", "yaw"):
            if name not in self.axis_map:
                continue
            num, inv = self.axis_map[name]
            v = apply_deadzone(js.axis(num), dz)
            if inv:
                v = -v
            self.channels[AXIS_TO_CH[name]] = clamp(MID + v * self.args.deflection)
        if "throttle" in self.axis_map:
            num, inv = self.axis_map["throttle"]
            tv = apply_deadzone(js.axis(num), dz)
            if inv:
                tv = -tv
            if self.args.throttle_mode == "direct":
                self.channels[CH_THROTTLE] = clamp(MID + tv * 500)
            else:
                self.channels[CH_THROTTLE] = clamp(
                    self.channels[CH_THROTTLE] + tv * self.args.throttle_rate * dt)

    def run_joystick(self):
        period = 1.0 / self.args.rate
        last = time.monotonic()
        next_draw = 0.0
        try:
            while True:
                loop_start = time.monotonic()
                dt = loop_start - last
                last = loop_start
                for num in self.js.poll():
                    action = self.button_action.get(num)
                    if action == "quit":
                        return
                    if action == "msp":
                        self.toggle("msp_next")
                    elif action:
                        self.toggle(action)
                self.joystick_axes(dt)
                self.send()
                if loop_start >= next_draw:
                    self.render()
                    next_draw = loop_start + 0.05
                sleep = period - (time.monotonic() - loop_start)
                if sleep > 0:
                    time.sleep(sleep)
        except KeyboardInterrupt:
            pass
        finally:
            self.safe_shutdown()
            self.js.close()
            sys.stdout.write("\nDisarmed. Bye.\n")

    def run_list(self):
        print(f"Reading {self.js.path}. Move sticks / press buttons; Ctrl-C to exit.\n")
        try:
            while True:
                self.js.poll()
                while self.js.raw_events:
                    etype, number, value = self.js.raw_events.pop(0)
                    kind = "AXIS" if (etype & ~Joystick.JS_EVENT_INIT) == \
                        Joystick.JS_EVENT_AXIS else "BTN "
                    init = " (init)" if etype & Joystick.JS_EVENT_INIT else ""
                    print(f"  {kind} #{number:<2} value={value:>7}{init}")
                time.sleep(0.02)
        except KeyboardInterrupt:
            print()

    # --- display -----------------------------------------------------------
    def render(self):
        a, c = self.args, self.channels
        mode = "gamepad" if self.js else "keyboard"
        lo, hi = self.msp_states[self.msp_state]
        lines = [
            f"Custom RC [{mode}]  ->  udp://{a.host}:{a.port}   (Ctrl-C to quit)",
            "-" * 66,
            f"  Roll {c[0]:>4}   Pitch {c[1]:>4}   Throttle {c[2]:>4}   Yaw {c[3]:>4}",
            f"  ARM(AUX{a.arm_aux}): {onoff(self.armed)}   "
            f"ANGLE(AUX{a.angle_aux}): {onoff(self.angle)}",
            f"  OVERRIDE(AUX{a.msp_aux}): state {self.msp_state + 1}/"
            f"{len(self.msp_states)}  value {self.msp_value()}  range {lo}-{hi}",
        ]
        if self.mirror_idx:
            m = self.mirror_idx
            lines.append(
                f"  MIRROR -> AUX{a.mirror_aux}..{a.mirror_aux + 3} "
                f"[R {c[m]} P {c[m+1]} Y {c[m+2]} T {c[m+3]}]")
        lines.append("-" * 66)
        if self.js:
            lines.append("  sticks fly · buttons: arm/angle/panic · msp button = next state")
        else:
            lines.append("  thr w/s  yaw a/d  pitch i/k  roll j/l  arm=SPACE  g=angle")
            lines.append("  m/n = next/prev override state   1-9 = pick state   x=panic  q=quit")
        out = []
        if self._status_lines:
            out.append(f"\033[{self._status_lines}A")
        for ln in lines:
            out.append("\033[2K" + ln + "\n")
        sys.stdout.write("".join(out))
        sys.stdout.flush()
        self._status_lines = len(lines)

    # --- entry -------------------------------------------------------------
    def run(self):
        if self.args.joystick:
            try:
                self.js = Joystick(self.args.joystick)
            except OSError as exc:
                sys.exit(f"custom_virtual_rc: cannot open {self.args.joystick}: {exc}")
            if self.args.list:
                self.run_list()
            else:
                self.run_joystick()
        else:
            if not sys.stdin.isatty():
                sys.exit("custom_virtual_rc: keyboard mode needs a TTY "
                         "(use --joystick for gamepad).")
            self.run_keyboard()


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="Custom virtual RC for Betaflight SITL "
                    "(stick mirroring + multi-state override switch).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--host", default="127.0.0.1", help="SITL address")
    p.add_argument("--port", type=int, default=9004, help="SITL RC UDP port")
    p.add_argument("--rate", type=float, default=100.0, help="send rate (Hz)")
    p.add_argument("--arm-aux", type=int, default=1, help="AUX channel to arm")
    p.add_argument("--angle-aux", type=int, default=2,
                   help="AUX channel for angle mode")
    p.add_argument("--msp-aux", type=int, default=3,
                   help="AUX channel for the multi-state override switch")
    p.add_argument("--msp-states", default="1000-1200,1201-1450,1451-1550,1551-2000",
                   help="override switch ranges 'lo-hi,lo-hi,...' (cycled with m/n)")
    p.add_argument("--mirror-aux", type=int, default=7,
                   help="first AUX of the roll/pitch/yaw/throttle mirror block "
                        "(0 disables). Default AUX7 -> AUX7..AUX10")
    p.add_argument("--deflection", type=int, default=400,
                   help="max stick deflection from centre (us)")
    p.add_argument("--throttle-step", type=int, default=25,
                   help="keyboard throttle change per key press (us)")

    g = p.add_argument_group("gamepad (Linux joystick API)")
    g.add_argument("--joystick", nargs="?", const="/dev/input/js0", default=None,
                   metavar="DEV", help="use a gamepad (default /dev/input/js0)")
    g.add_argument("--list", action="store_true",
                   help="with --joystick: print axis/button numbers and exit")
    g.add_argument("--axes", default="roll=3,pitch=4i,throttle=1i,yaw=0",
                   help="axis map name=number[i] (i inverts)")
    g.add_argument("--buttons", default="arm=0,panic=1,angle=2,msp=3,quit=6",
                   help="button map name=number (names: arm angle msp panic quit)")
    g.add_argument("--throttle-mode", choices=("incremental", "direct"),
                   default="incremental", help="gamepad throttle behaviour")
    g.add_argument("--throttle-rate", type=float, default=600.0,
                   help="incremental throttle change at full stick (us/s)")
    g.add_argument("--deadzone", type=float, default=0.08,
                   help="gamepad stick centre deadzone (0..1)")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    CustomVirtualRC(args).run()


if __name__ == "__main__":
    main()

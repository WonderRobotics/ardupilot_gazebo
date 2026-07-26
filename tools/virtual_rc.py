#!/usr/bin/env python3
"""Virtual RC receiver for Betaflight SITL (keyboard or gamepad).

Streams an ``rc_packet`` over UDP to Betaflight SITL (default 127.0.0.1:9004),
exactly like a hardware receiver. The first packet switches the SITL to the
UDP RC provider, after which the AUX channels drive the normal Betaflight mode
logic (arm, angle, MSP OVERRIDE, ...).

Wire format (betaflight/src/platform/SIMULATOR/target/SITL/target.h):

    typedef struct {
        double   timestamp;      // seconds
        uint16_t channels[16];   // us, AETR + AUX order
    } rc_packet;                 // 40 bytes, little-endian

Channel order is AETR:
    ch0 = Roll (Aileron)   ch1 = Pitch (Elevator)
    ch2 = Throttle         ch3 = Yaw (Rudder)
    ch4.. = AUX1, AUX2, ...

MSP OVERRIDE workflow (mirrors a real transmitter):
    * A configurable AUX switch (``--msp-aux``, default AUX3) toggles between
      low (1000) and high (2000).
    * On the FC, bind the "MSP OVERRIDE" mode (box id 50) to that AUX high, and
      set ``msp_override_channels_mask`` to the channels the companion computer
      should drive. See the README for the exact CLI.
    * Flip the switch high (key 'm' / a gamepad button) and the FC replaces the
      masked channels with the values your C++ app sends via MSP_SET_RAW_RC
      (MSP serial = TCP 5761). Flip it low to hand control back to the sticks.

--------------------------------------------------------------------------
KEYBOARD mode (default). OS key-repeat holds a stick deflected:
    throttle w/s   yaw a/d   pitch i/k   roll j/l
    SPACE arm    g angle    m msp-override    x panic    c centre    q quit

GAMEPAD mode ( --joystick [/dev/input/jsN] ). Uses the Linux joystick API:
    sticks fly the aircraft; buttons toggle arm / angle / msp-override / panic.
    Run with --list first to discover your controller's axis/button numbers,
    then map them with --axes / --buttons.
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

# AETR stick channel indices.
CH_ROLL, CH_PITCH, CH_THROTTLE, CH_YAW = 0, 1, 2, 3
AXIS_TO_CH = {"roll": CH_ROLL, "pitch": CH_PITCH,
              "throttle": CH_THROTTLE, "yaw": CH_YAW}


def clamp(v, lo=LOW, hi=HIGH):
    return max(lo, min(hi, int(v)))


def onoff(b):
    return "\033[32mON \033[0m" if b else "\033[90moff\033[0m"


def aux_to_index(aux_number):
    """AUX1 -> channel index 4, AUX2 -> 5, ..."""
    return 3 + aux_number


def apply_deadzone(v, dz):
    """Map [-1,1] with a centre deadzone to a smooth [-1,1]."""
    if abs(v) < dz:
        return 0.0
    return (v - math.copysign(dz, v)) / (1.0 - dz)


def parse_map(spec, allow_invert):
    """Parse "roll=3,pitch=4i,..." into {name: (number, invert)}.

    A trailing 'i' inverts the axis (only when allow_invert)."""
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
        self.raw_events = []  # for --list

    def poll(self):
        """Read all pending events; return list of button numbers just pressed."""
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


class VirtualRC:
    def __init__(self, args):
        self.args = args
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (args.host, args.port)

        self.arm_idx = aux_to_index(args.arm_aux)
        self.angle_idx = aux_to_index(args.angle_aux)
        self.msp_idx = aux_to_index(args.msp_aux)

        self.channels = [LOW] * NUM_CHANNELS
        self.channels[CH_ROLL] = MID
        self.channels[CH_PITCH] = MID
        self.channels[CH_THROTTLE] = LOW
        self.channels[CH_YAW] = MID

        self.armed = False
        self.angle = False
        self.msp = False
        self.start = time.monotonic()
        self._status_lines = 0

        # Gamepad maps (parsed lazily; only needed in joystick mode).
        self.axis_map = parse_map(args.axes, allow_invert=True)
        button_map = parse_map(args.buttons, allow_invert=False)
        self.button_action = {num: name for name, (num, _) in button_map.items()}
        self.js = None

    # --- shared actions ----------------------------------------------------
    def toggle(self, action):
        if action == "arm":
            self.armed = not self.armed
        elif action == "angle":
            self.angle = not self.angle
        elif action == "msp":
            self.msp = not self.msp
        elif action == "panic":
            self.armed = False
            self.channels[CH_THROTTLE] = LOW
        elif action == "center":
            self.channels[CH_ROLL] = MID
            self.channels[CH_PITCH] = MID
            self.channels[CH_YAW] = MID

    def apply_switches(self):
        self.channels[self.arm_idx] = HIGH if self.armed else LOW
        self.channels[self.angle_idx] = HIGH if self.angle else LOW
        self.channels[self.msp_idx] = HIGH if self.msp else LOW

    def send(self):
        self.apply_switches()
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
            self.toggle("msp")
        elif key == "x":
            self.toggle("panic")
        elif key == "c":
            self.toggle("center")
        elif key in ("q", "\x03"):
            return False
        return True

    def decay(self):
        """Self-centre the momentary axes toward MID (keyboard mode)."""
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
            else:  # incremental: stick sets rate of change, value latches
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
                    if action:
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
        print(f"Reading {self.js.path}. Move each stick and press each button")
        print("to learn its number, then map with --axes / --buttons.")
        print("Ctrl-C to exit.\n")
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
        lines = [
            f"Virtual RC [{mode}]  ->  udp://{a.host}:{a.port}   (Ctrl-C to quit)",
            "-" * 62,
            f"  Roll {c[0]:>4}   Pitch {c[1]:>4}   Throttle {c[2]:>4}   Yaw {c[3]:>4}",
            f"  ARM(AUX{a.arm_aux}): {onoff(self.armed)}   "
            f"ANGLE(AUX{a.angle_aux}): {onoff(self.angle)}   "
            f"MSP-OVR(AUX{a.msp_aux}): {onoff(self.msp)}",
            "-" * 62,
        ]
        if self.js:
            lines.append("  sticks fly · buttons: arm/angle/msp/panic (see --buttons)")
        else:
            lines.append("  thr w/s  yaw a/d  pitch i/k  roll j/l  arm=SPACE")
            lines.append("  g=angle  m=msp-override  x=panic  c=centre  q=quit")
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
                sys.exit(f"virtual_rc: cannot open {self.args.joystick}: {exc}")
            if self.args.list:
                self.run_list()
            else:
                self.run_joystick()
        else:
            if not sys.stdin.isatty():
                sys.exit("virtual_rc: keyboard mode needs a TTY "
                         "(use --joystick for gamepad).")
            self.run_keyboard()


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="Virtual RC receiver (transmitter) for Betaflight SITL.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--host", default="127.0.0.1", help="SITL address")
    p.add_argument("--port", type=int, default=9004, help="SITL RC UDP port")
    p.add_argument("--rate", type=float, default=100.0, help="send rate (Hz)")
    p.add_argument("--arm-aux", type=int, default=1, help="AUX channel to arm")
    p.add_argument("--angle-aux", type=int, default=2,
                   help="AUX channel for angle (self-level) mode")
    p.add_argument("--msp-aux", type=int, default=3,
                   help="AUX channel to toggle MSP OVERRIDE mode")
    p.add_argument("--deflection", type=int, default=400,
                   help="max stick deflection from centre (us)")
    p.add_argument("--throttle-step", type=int, default=25,
                   help="keyboard throttle change per key press (us)")

    g = p.add_argument_group("gamepad (Linux joystick API)")
    g.add_argument("--joystick", nargs="?", const="/dev/input/js0", default=None,
                   metavar="DEV",
                   help="use a gamepad instead of the keyboard "
                        "(default device /dev/input/js0)")
    g.add_argument("--list", action="store_true",
                   help="with --joystick: print axis/button numbers and exit")
    g.add_argument("--axes", default="roll=3,pitch=4i,throttle=1i,yaw=0",
                   help="axis map name=number[i] (i inverts); Xbox-style default")
    g.add_argument("--buttons", default="arm=0,panic=1,angle=2,msp=3,center=7,quit=6",
                   help="button map name=number; names: arm angle msp panic "
                        "center quit")
    g.add_argument("--throttle-mode", choices=("incremental", "direct"),
                   default="incremental",
                   help="incremental: stick sets throttle rate (latches, good "
                        "for spring-centred gamepads); direct: stick = throttle")
    g.add_argument("--throttle-rate", type=float, default=600.0,
                   help="incremental throttle change at full stick (us/s)")
    g.add_argument("--deadzone", type=float, default=0.08,
                   help="gamepad stick centre deadzone (0..1)")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    VirtualRC(args).run()


if __name__ == "__main__":
    main()

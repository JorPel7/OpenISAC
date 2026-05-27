#!/usr/bin/env python3
"""
channel_sim_control.py — Runtime parameter control for ChannelSimulator.

Uses the SAME UDP packet format as plot_sensing_fast.py / send_control_command().
Point it at the channel sim's control_port (default 9998).

Usage:
  python3 channel_sim_control.py --ip 127.0.0.1 --port 9998
  Then type commands interactively, e.g.:
    velocity 15.5
    distance 200
    md_amp 0.2
    md_freq 25
    status
    quit

Can also be imported and used programmatically:
    from channel_sim_control import ChannelSimController
    c = ChannelSimController("127.0.0.1", 9998)
    c.set_velocity(15.5)
    c.set_micro_doppler(amp=0.2, freq=25.0)
"""

import argparse
import socket
import struct
import sys

# ── Packet format ─────────────────────────────────────────────────────────────
# Identical to plot_sensing_fast.py → send_control_command():
#   struct.pack("!4s4si", header, cmd_id, value)
#
# Fixed-point encoding for the channel-specific commands:
#   VEL   velocity × 100    signed   (1550 = 15.50 m/s, -200 = -2.00 m/s)
#   DIST  distance × 10     unsigned (1000 = 100.0 m)
#   MDAF  md_amp × 10000    signed   (2000 = 0.2000 rad)
#   MDFF  md_freq × 100     signed   (2500 = 25.00 Hz)

CMD_HEADER = b"CMD "

def _pack(cmd_id: bytes, value: int) -> bytes:
    return struct.pack("!4s4si", CMD_HEADER, cmd_id, int(value))


class ChannelSimController:
    def __init__(self, ip: str = "127.0.0.1", port: int = 9998):
        self.ip   = ip
        self.port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _send(self, cmd_id: bytes, raw_value: int) -> None:
        self._sock.sendto(_pack(cmd_id, raw_value), (self.ip, self.port))

    def set_velocity(self, velocity_m_s: float) -> None:
        """Set target radial velocity (m/s). Positive = approaching."""
        self._send(b"VEL ", int(round(velocity_m_s * 100)))
        print(f"[CMD] velocity = {velocity_m_s:.2f} m/s")

    def set_distance(self, distance_m: float) -> None:
        """Set target distance (m). Informational — used for future delay extension."""
        self._send(b"DIST", int(round(distance_m * 10)))
        print(f"[CMD] distance = {distance_m:.1f} m")

    def set_micro_doppler(self, amp: float, freq: float) -> None:
        """Set micro-Doppler parameters. amp in rad, freq in Hz."""
        self._send(b"MDAF", int(round(amp * 10000)))
        self._send(b"MDFF", int(round(freq * 100)))
        print(f"[CMD] micro_doppler: amp={amp:.4f} rad  freq={freq:.2f} Hz")

    def set_md_amp(self, amp_rad: float) -> None:
        self._send(b"MDAF", int(round(amp_rad * 10000)))
        print(f"[CMD] micro_doppler_amp = {amp_rad:.4f} rad")

    def set_md_freq(self, freq_hz: float) -> None:
        self._send(b"MDFF", int(round(freq_hz * 100)))
        print(f"[CMD] micro_doppler_freq = {freq_hz:.2f} Hz")

    def close(self) -> None:
        self._sock.close()


# ── Interactive CLI ────────────────────────────────────────────────────────────

HELP = """
Commands:
  velocity  <m/s>          — radial velocity (positive = approaching)
  distance  <m>            — target distance (informational)
  md_amp    <rad>          — micro-Doppler phase deviation
  md_freq   <Hz>           — micro-Doppler modulation rate
  status                   — print current parameter state
  help                     — show this message
  quit / exit / Ctrl+C     — exit
"""

def interactive(ctrl: ChannelSimController) -> None:
    # Local shadow to print 'status' without a round-trip to the C++ process
    state = {"velocity": 0.0, "distance": 100.0, "md_amp": 0.0, "md_freq": 10.0}
    print(HELP)
    while True:
        try:
            line = input(f"channel_sim [{ctrl.ip}:{ctrl.port}]> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            break
        elif cmd == "help":
            print(HELP)
        elif cmd == "status":
            print(f"  velocity  = {state['velocity']:.2f} m/s")
            print(f"  distance  = {state['distance']:.1f} m")
            print(f"  md_amp    = {state['md_amp']:.4f} rad")
            print(f"  md_freq   = {state['md_freq']:.2f} Hz")
        elif cmd in ("velocity", "vel") and len(parts) == 2:
            v = float(parts[1])
            ctrl.set_velocity(v)
            state["velocity"] = v
        elif cmd in ("distance", "dist") and len(parts) == 2:
            d = float(parts[1])
            ctrl.set_distance(d)
            state["distance"] = d
        elif cmd in ("md_amp", "mdaf") and len(parts) == 2:
            a = float(parts[1])
            ctrl.set_md_amp(a)
            state["md_amp"] = a
        elif cmd in ("md_freq", "mdff") and len(parts) == 2:
            f = float(parts[1])
            ctrl.set_md_freq(f)
            state["md_freq"] = f
        else:
            print(f"  Unknown command: {line!r} — type 'help'")


def main() -> None:
    parser = argparse.ArgumentParser(description="ChannelSimulator runtime control")
    parser.add_argument("--ip",   default="127.0.0.1", help="Channel sim host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9998, help="Control UDP port (default: 9998)")
    # One-shot mode
    parser.add_argument("--velocity",  type=float, default=None, metavar="M/S")
    parser.add_argument("--distance",  type=float, default=None, metavar="M")
    parser.add_argument("--md-amp",    type=float, default=None, metavar="RAD")
    parser.add_argument("--md-freq",   type=float, default=None, metavar="HZ")
    args = parser.parse_args()

    ctrl = ChannelSimController(args.ip, args.port)

    one_shot = any(v is not None for v in (args.velocity, args.distance,
                                            args.md_amp, args.md_freq))
    if one_shot:
        if args.velocity  is not None: ctrl.set_velocity(args.velocity)
        if args.distance  is not None: ctrl.set_distance(args.distance)
        if args.md_amp    is not None: ctrl.set_md_amp(args.md_amp)
        if args.md_freq   is not None: ctrl.set_md_freq(args.md_freq)
    else:
        interactive(ctrl)

    ctrl.close()


if __name__ == "__main__":
    main()

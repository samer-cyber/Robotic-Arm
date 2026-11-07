#!/usr/bin/env python3
"""Keyboard jog - drive one joint at a time from the terminal.

    python jog.py                 # auto-detect port
    python jog.py --port COM7
    python jog.py --simulate

Keys
    1..6        select joint
    up / down   jog selected joint + / -
    left/right  smaller / bigger step
    [ ]         slower / faster feed
    e           toggle motors
    z           teach selected joint = 0
    Z           teach ALL joints = 0
    s           save config + positions to flash
    /           type a raw firmware command
    space       E-STOP
    q           quit
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from armlink import connect, list_serial_ports          # noqa: E402

if sys.platform == "win32":
    import msvcrt

    def key_ready():
        return msvcrt.kbhit()

    def key_read():
        ch = msvcrt.getch()
        if ch in (b"\x00", b"\xe0"):
            code = msvcrt.getch()
            return {b"H": "UP", b"P": "DOWN", b"K": "LEFT", b"M": "RIGHT"}.get(code, "")
        try:
            return ch.decode("utf-8", "ignore")
        except Exception:
            return ""
else:
    import select
    import termios
    import tty
    _old = None

    def raw_mode(on):
        global _old
        fd = sys.stdin.fileno()
        if on:
            _old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        elif _old:
            termios.tcsetattr(fd, termios.TCSADRAIN, _old)

    def key_ready():
        return select.select([sys.stdin], [], [], 0)[0] != []

    def key_read():
        ch = sys.stdin.read(1)
        if ch == "\x1b" and select.select([sys.stdin], [], [], 0.05)[0]:
            seq = sys.stdin.read(2)
            return {"[A": "UP", "[B": "DOWN", "[D": "LEFT", "[C": "RIGHT"}.get(seq, "ESC")
        return ch


STEPS = [0.1, 0.5, 1, 2, 5, 10, 20, 45, 90]


def main():
    ap = argparse.ArgumentParser(description="keyboard jog for the arm")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--simulate", action="store_true")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    if args.list:
        for dev, desc in list_serial_ports():
            print(f"{dev:10s} {desc}")
        return

    try:
        arm = connect(args.port, args.baud, simulate=args.simulate)
    except Exception as e:
        print(f"could not connect: {e}")
        for dev, desc in list_serial_ports():
            print(f"  {dev:10s} {desc}")
        print("\n  * close any serial monitor holding the port")
        print("  * or pass --port COMx, or --simulate")
        raise SystemExit(1)

    names = [j.name for j in arm.meta]
    if not names:
        raise SystemExit("controller reported no joints")
    print(__doc__)
    print("joints:", ", ".join(f"{i+1}={n}" for i, n in enumerate(names)))

    sel, si, feed, msg = 0, 4, 0.4, "ready"
    if sys.platform != "win32":
        raw_mode(True)

    try:
        while True:
            while key_ready():
                k = key_read()
                if k in ("q", "Q", "ESC"):
                    raise KeyboardInterrupt

                if k in "123456" and int(k) <= len(names):
                    sel = int(k) - 1
                    msg = f"joint {names[sel]}"
                elif k == "UP" or k == "DOWN":
                    d = STEPS[si] * (1 if k == "UP" else -1)
                    r = arm.jog(sel, d, feed)
                    msg = "ok" if r else f"{r.err_name}: {r.err_detail}"
                elif k == "LEFT":
                    si = max(0, si - 1)
                elif k == "RIGHT":
                    si = min(len(STEPS) - 1, si + 1)
                elif k == "[":
                    feed = max(0.05, round(feed - 0.05, 2))
                elif k == "]":
                    feed = min(1.0, round(feed + 0.05, 2))
                elif k == "e":
                    st = arm.state()
                    r = arm.enable(not st.enabled)
                    msg = "motors on" if not st.enabled else "motors off"
                    if not r:
                        msg = f"{r.err_name}: {r.err_detail}"
                elif k == "z":
                    arm.teach(sel, 0)
                    msg = f"{names[sel]} = 0"
                elif k == "Z":
                    arm.teach("all", 0)
                    msg = "all joints = 0"
                elif k == "s":
                    arm.save()
                    msg = "saved"
                elif k == " ":
                    arm.estop()
                    msg = "*** E-STOP *** (press e twice to recover)"
                elif k == "/":
                    if sys.platform != "win32":
                        raw_mode(False)
                    sys.stdout.write("\n> ")
                    sys.stdout.flush()
                    line = sys.stdin.readline().strip()
                    if sys.platform != "win32":
                        raw_mode(True)
                    if line:
                        r = arm.cmd(line, timeout=60)
                        print()
                        for l in r.raw:
                            print("   " + l)
                        msg = str(r)

            st = arm.state()
            pos = "  ".join(
                f"{'>' if i == sel else ' '}{j.name} {j.position:8.2f}"
                for i, j in enumerate(st.joints))
            sys.stdout.write(
                f"\r\033[K{pos} | step {STEPS[si]:>5} feed {feed:.2f} | "
                f"{'EN' if st.enabled else '--'}"
                f"{' MOVING' if st.moving else '       '}"
                f"{' ESTOP' if st.estopped else ''} | {msg[:36]}")
            sys.stdout.flush()
            time.sleep(0.08)

    except KeyboardInterrupt:
        pass
    finally:
        if sys.platform != "win32":
            raw_mode(False)
        print("\nstopping")
        try:
            arm.stop()
        except Exception:
            pass
        arm.close()


if __name__ == "__main__":
    main()

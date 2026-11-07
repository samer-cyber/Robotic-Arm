#!/usr/bin/env python3
"""Servo test rig - J4 / J5 / J6 + gripper.

    python servo_test.py                  # auto-detect port
    python servo_test.py --port COM7
    python servo_test.py --list

These are OPEN LOOP. The firmware reports what it last COMMANDED, never what
the horn is actually doing - a servo stalled against a bracket looks exactly
like one that arrived. If a number here disagrees with your eyes, believe your
eyes and go find the mechanical stop.

Commands
    j4 <deg>      set a joint. 0 = middle of travel, so a 270 deg servo
    j5 <deg>      runs -135..+135 (soft-limited to +/-90 until you open it up)
    j6 <deg>
    grip <0-100>  jaw percent. also: grip open / grip close
    +  -          nudge the last joint you touched by `step`
    step <deg>    change the nudge size (default 5)
    sweep <j>     walk a joint end to end in small steps - THE way to find
                  where it really hits something. ctrl-C stops it.
    center        all three wrist joints to 0
    park          everything to its park angle
    limp          stop the pulses - servos go slack, safe to handle
    live          resume pulses
    desk [z [m]]  work-surface floor and live clearance under the whole arm
    sv            the servo table
    <anything>    passed straight to the firmware
    q             quit
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from armlink import connect, list_serial_ports          # noqa: E402

WRIST = ("j4", "j5", "j6")


def show(reply) -> None:
    for line in reply.raw:
        print("   " + line)
    if not reply:
        print(f"   !! {reply}")


def sweep(arm, name: str, lo: float, hi: float, step: float, dwell: float) -> None:
    """End to end and back. Watch the arm, not the screen."""
    print(f"   sweeping {name} {lo:+.0f} -> {hi:+.0f} -> {lo:+.0f}, {step:.0f} deg steps")
    print("   ctrl-C to stop")
    seq = list(_frange(lo, hi, step)) + list(_frange(hi, lo, -step))
    try:
        for a in seq:
            r = arm.cmd(f"sv {name} {a:.1f}")
            flag = "" if r else "  <- REJECTED"
            print(f"\r   {name} {a:+7.1f}{flag}   ", end="", flush=True)
            time.sleep(dwell)
        print("\n   done")
    except KeyboardInterrupt:
        print("\n   stopped - servo left where it was")


def _frange(a: float, b: float, s: float):
    if s == 0:
        return
    n = int(abs((b - a) / s)) + 1
    for i in range(n):
        yield a + i * s


def main() -> int:
    ap = argparse.ArgumentParser(description="servo tester for J4/J5/J6 + gripper")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=500000)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--simulate", action="store_true")
    args = ap.parse_args()

    if args.list:
        for dev, desc in list_serial_ports():
            print(f"{dev:10s} {desc}")
        return 0

    try:
        arm = connect(args.port, args.baud, simulate=args.simulate)
    except Exception as e:
        print(f"could not connect: {e}")
        for dev, desc in list_serial_ports():
            print(f"  {dev:10s} {desc}")
        print("\n  * close any serial monitor holding the port")
        print("  * or pass --port COMx")
        return 1

    print(__doc__)
    show(arm.cmd("sv"))
    print("   NOTE servo power must be its own 5-6 V supply, grounded to the ESP32.")
    print("   Four of these can pull well over 5 A stalled; the board's 5V pin cannot.\n")

    last = "j4"
    step = 5.0

    try:
        while True:
            try:
                raw = input(f"[{last} step {step:g}] > ").strip()
            except EOFError:
                break
            if not raw:
                continue
            parts = raw.split()
            head = parts[0].lower()
            rest = parts[1:]

            try:
                if head in ("q", "quit", "exit"):
                    break

                elif head in WRIST and rest:
                    last = head
                    show(arm.cmd(f"sv {head} {float(rest[0])}"))

                elif head in WRIST:
                    last = head
                    show(arm.cmd(f"sv {head}"))

                elif head == "grip":
                    show(arm.cmd("grip " + (rest[0] if rest else "")))

                elif head in ("+", "-"):
                    r = arm.cmd(f"sv {last}")
                    cur = _current(r, last)
                    if cur is None:
                        print("   could not read current angle")
                        continue
                    show(arm.cmd(f"sv {last} {cur + (step if head == '+' else -step):.1f}"))

                elif head == "step" and rest:
                    step = abs(float(rest[0]))

                elif head == "sweep":
                    name = (rest[0].lower() if rest else last)
                    lo, hi = _limits(arm, name)
                    if lo is None:
                        print("   no such servo")
                        continue
                    last = name
                    sweep(arm, name, lo, hi, step if step else 5.0, 0.25)

                elif head == "center":
                    for j in WRIST:
                        show(arm.cmd(f"sv {j} 0"))

                elif head == "park":
                    show(arm.cmd("svpark"))

                elif head == "limp":
                    show(arm.cmd("svoff"))

                elif head == "live":
                    show(arm.cmd("svattach"))

                elif head == "desk":
                    show(arm.cmd("desk " + " ".join(rest)))

                else:
                    show(arm.cmd(raw, timeout=30))

            except ValueError:
                print("   that needs a number")
            except Exception as e:
                print(f"   {type(e).__name__}: {e}")

    except KeyboardInterrupt:
        pass
    finally:
        print("\nleaving servos where they are (use `limp` first if you want them slack)")
        arm.close()
    return 0


def _current(reply, name: str):
    """Pull the angle back out of '# J4 12.0 deg  1588 us' or the sv table."""
    for line in reply.raw:
        t = line.replace("#", " ").split()
        if len(t) >= 2 and t[0].lower() == name.lower():
            try:
                return float(t[1])
            except ValueError:
                pass
    return None


def _limits(arm, name: str):
    for line in arm.cmd("sv").raw:
        t = line.replace("#", " ").split()
        # id name deg us min..max range trim inv pin
        if len(t) >= 6 and t[1].lower() == name.lower():
            for tok in t:
                if ".." in tok and tok[0] in "-0123456789":
                    try:
                        lo, hi = tok.split("..")
                        return float(lo), float(hi)
                    except ValueError:
                        pass
    return None, None


if __name__ == "__main__":
    sys.exit(main())

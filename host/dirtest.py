#!/usr/bin/env python3
"""Direction and coordination test for the UART shaft-bit direction control.

Run this BEFORE the first /vla/execute with dry_run:false.

What it is actually testing
---------------------------
Direction is no longer a GPIO. It is the TMC2209 GCONF.shaft bit, written over
the UART bus, and `diag joint N` only ever proved that for ONE joint at +/-5
degrees. A coordinated move is a different code path: Arm::moveAll() sets all
three shaft bits in a pre-pass and then starts three axes together. That path
has never run on hardware.

So this goes in two phases, and the order matters:

  Phase 1  one joint at a time, out and back. If a joint is wired or
           configured backwards, this is where you see it, with nothing else
           moving to confuse the picture.
  Phase 2  all three at once, with MIXED directions - J1 positive, J2 and J3
           negative. That is the case that exercises the pre-pass properly:
           one shaft bit set one way, two the other, all in the same move.

Everything here is open loop. The step counter ALWAYS comes back to where it
started, because the firmware is just counting what it sent. That number proves
nothing. The evidence is the arm itself: watch it, and answer the prompts
honestly. A joint running backwards will still report a perfect return.

    python3 "/mnt/d/Samer/Projects/Robotic Arm/host/dirtest.py"
    python3 .../dirtest.py --port /dev/ttyUSB0 --feed 0.08
    python3 .../dirtest.py --simulate          # no hardware, checks the logic

Stop W2 first. One program per serial port.
"""

from __future__ import annotations

import argparse
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0] if "/" in __file__ else ".")

from armlink import connect, ArmError            # noqa: E402

JOINTS = ("J1", "J2", "J3")

# Where the arm should be before this starts: the candle pose.
CANDLE = {"J1": 0.0, "J2": 90.0, "J3": 0.0}

# Phase 2 target. Deliberately mixed-sign from the candle pose:
#   J1  0 -> +10   positive
#   J2 90 -> +85   negative
#   J3  0 -> -10   negative
COMBINED = {"J1": 10.0, "J2": 85.0, "J3": -10.0}

# Which way each joint should visibly turn, going from CANDLE to COMBINED.
EXPECT = {
    "J1": "base rotates +10 deg (counter-clockwise seen from above)",
    "J2": "shoulder tips 5 deg FORWARD, away from straight up",
    "J3": "elbow bends 10 deg, forearm leaves the straight-up line",
}


def phase1_target(arm, name: str, base: float, step: float):
    """Pick a test target for one joint that its own limits actually allow.

    Not cosmetic: at the candle pose J3 sits at 0.0, which IS its upper limit
    ([-150, 0]). Nudging it +5 would be rejected as out-of-range and would look
    exactly like a direction fault. Ask the firmware what the limits are, and
    move whichever way there is room.
    """
    meta = {j.name: j for j in arm.meta}
    m = meta.get(name)
    if m is None:
        # Refuse rather than assume. Guessing +/-180 here is what stepped J3
        # to +5 when its real limit is 0: `info` had come back short, so the
        # joint was simply absent from meta and the fallback took over.
        return None, None
    lo, hi = m.min_deg, m.max_deg
    if base + step <= hi - 1e-6:
        return base + step, "+"
    if base - step >= lo + 1e-6:
        return base - step, "-"
    return None, None


def ask(prompt: str) -> bool:
    try:
        return input(f"{prompt} [y/N/q] ").strip().lower().startswith("y")
    except (EOFError, KeyboardInterrupt):
        return False


def show(arm) -> dict:
    st = arm.state()
    # ALL joints, not state.by_name() - that filters to `usable`, so a joint
    # whose row got lost on the wire silently became NaN and looked like a
    # broken encoder rather than a dropped line.
    pos = {j.name: j.position for j in st.joints}
    cells = []
    for n in JOINTS:
        cells.append(f"{n}={pos[n]:+8.3f}" if n in pos else f"{n}=  MISSING")
    print("    " + "  ".join(cells)
          + f"   enabled={st.enabled} moving={st.moving} estop={st.estopped}")
    missing = [n for n in JOINTS if n not in pos]
    if missing:
        print("    !! " + ", ".join(missing) + " absent from the reply - the "
              "link dropped rows. This is NOT a joint fault.")
    return pos


def move(arm, target: dict, feed: float) -> bool:
    r = arm.goto_named(target, feed=feed)
    if not r:
        print(f"    REFUSED: {r.err_name} {r.err_detail}")
        return False
    if not arm.wait_idle(timeout=60.0):
        print("    TIMED OUT waiting for idle - stopping")
        arm.stop()
        return False
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=None, help="default: auto-detect")
    ap.add_argument("--feed", type=float, default=0.08)
    ap.add_argument("--step", type=float, default=5.0,
                    help="phase 1 amplitude, degrees")
    ap.add_argument("--settle", type=float, default=2.0,
                    help="seconds to wait after opening the port, for a "
                         "possible ESP32 auto-reset to finish booting")
    ap.add_argument("--simulate", action="store_true")
    a = ap.parse_args()

    print(__doc__.split("Run this")[0].strip())
    print()

    try:
        arm = connect(port=a.port, simulate=a.simulate, settle=a.settle)
    except Exception as exc:
        print(f"could not open the arm: {exc}")
        print("  - is W2 (vla_commander_node) still running? it holds the port")
        print("  - is control.py open on Windows?")
        print("  - fuser -v /dev/ttyUSB0   will name whoever has it")
        return 1

    names = [j.name for j in arm.meta]
    print(f"connected. firmware reports joints: {', '.join(names) or '(none!)'}")
    if not all(n in names for n in JOINTS):
        print("!! J1..J3 are not all reported. Fix that before moving anything.")
        print("   `info` was retried 3 times and still came back short.")
        print("   If the board had just reset, try a longer settle:")
        print(f"     --settle 4.0     (currently {a.settle:.1f}s)")
        print("   Otherwise check the link directly: pio device monitor, then type info")
        return 1

    print("\nstarting state:")
    start = show(arm)

    off = {n: abs(start.get(n, 999) - CANDLE[n]) for n in JOINTS}
    if any(v > 2.0 for v in off.values()):
        print(f"\n!! Not in the candle pose (expected {CANDLE}).")
        print("   Every move below is relative to where the firmware THINKS it is.")
        print("   If that belief is wrong, so is the motion. Teach it first.")
        if not ask("   Continue anyway?"):
            return 1

    print("\nCLEAR THE WORKSPACE. Move the blue box and anything else out of reach.")
    if not ask("Workspace clear and you are ready to energise?"):
        print("aborted, nothing was enabled")
        return 0

    if not arm.enable(True):
        print("could not enable the drivers")
        return 1
    print("drivers ENABLED - the arm is live from here on. Ctrl-C stops the script.")

    failures = []
    try:
        # ---------------- phase 1: one joint at a time --------------------
        print(f"\n=== PHASE 1: each joint alone, +{a.step:.0f} deg then back ===")
        for n in JOINTS:
            goal, sign = phase1_target(arm, n, CANDLE[n], a.step)
            if goal is None:
                print(f"\n  {n}: no room within its limits - skipped")
                failures.append(f"{n} could not be tested, limits too tight")
                continue
            tgt = dict(CANDLE)
            tgt[n] = goal
            print(f"\n  {n}: {CANDLE[n]:+.1f} -> {goal:+.1f} deg  ({sign} direction)")
            if not move(arm, tgt, a.feed):
                failures.append(f"{n} outbound move refused")
                continue
            show(arm)
            good = ask(f"  did {n} - and ONLY {n} - move, in the {sign} direction?")
            if not good:
                failures.append(f"{n} direction or isolation wrong")
            if not move(arm, dict(CANDLE), a.feed):
                failures.append(f"{n} return move refused")
            show(arm)

        # ---------------- phase 2: all three, mixed signs -----------------
        print("\n=== PHASE 2: all three together, mixed directions ===")
        for n in JOINTS:
            print(f"  {n}: {CANDLE[n]:+.1f} -> {COMBINED[n]:+.1f} deg   ({EXPECT[n]})")
        if ask("\n  run the combined move?"):
            if move(arm, dict(COMBINED), a.feed):
                show(arm)
                if not ask("  did all three move together, each the right way?"):
                    failures.append("combined move: wrong direction on at least one joint")
                if move(arm, dict(CANDLE), a.feed):
                    show(arm)
                    if not ask("  is the arm PHYSICALLY back in the candle pose?"):
                        failures.append("did not return to candle - counter and reality disagree")
                else:
                    failures.append("return to candle refused")
            else:
                failures.append("combined move refused")
    except ArmError as exc:
        print(f"\narm error: {exc}")
        failures.append(str(exc))
    except KeyboardInterrupt:
        print("\ninterrupted - stopping")
        try:
            arm.stop()
        except Exception:
            pass
    finally:
        try:
            arm.enable(False)
            print("\ndrivers disabled.")
        except Exception:
            pass

    print("\n" + "=" * 62)
    if failures:
        print("FAILURES:")
        for f in failures:
            print(f"  - {f}")
        print("\nA joint turning the wrong way is a config fix, not a code fix:")
        print("    set J<n> invert 1")
        print("    save")
        print("then re-run this. Do NOT run /vla/execute until this passes clean.")
    else:
        print("All checks passed. Coordinated direction control works on hardware.")
        print("You are clear to run /vla/execute with dry_run: false.")
    try:
        arm.close()
    except Exception:
        pass
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())

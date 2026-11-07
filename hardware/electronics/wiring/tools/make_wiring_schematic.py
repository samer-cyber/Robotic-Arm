#!/usr/bin/env python3
"""Generate the as-built wiring schematic (KiCad 9) from one pin table.

    python hardware/electronics/wiring/tools/make_wiring_schematic.py

Writes hardware/electronics/wiring/arm-wiring.kicad_sch (+ a minimal project).
Check it with:
    kicad-cli sch erc hardware/electronics/wiring/arm-wiring.kicad_sch

The pin numbers below were read off the controller itself (`pins` on
firmware 1.2.0, 2026-11-07), not copied from older docs - those disagreed.
Edit PINS and re-run if the wiring changes; do not hand-edit the output.

Modules (ESP32 DevKit, TMC2209 StepStick) are drawn as blocks with only the
pins that matter. Clone boards put pins in different physical places, so the
GPIO NUMBER is what to wire by, not the position on the drawing.
"""
from __future__ import annotations

import math
import uuid
from pathlib import Path

def _kicad_symbols() -> Path:
    """KiCad's stock symbol libraries: env var first, then the usual installs."""
    import os
    for var in ("KICAD9_SYMBOL_DIR", "KICAD_SYMBOL_DIR"):
        if os.environ.get(var):
            return Path(os.environ[var])
    for c in (Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/KiCad/9.0/share/kicad/symbols",
              Path("C:/Program Files/KiCad/9.0/share/kicad/symbols"),
              Path("/usr/share/kicad/symbols"),
              Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols")):
        if (c / "Device.kicad_sym").exists():
            return c
    raise SystemExit("KiCad symbol libraries not found - set KICAD9_SYMBOL_DIR")


KICAD_SYM = _kicad_symbols()
OUT = Path(__file__).resolve().parents[1]
NAME = "arm-wiring"
ROOT_UUID = str(uuid.uuid5(uuid.NAMESPACE_URL, "robotic-arm/arm-wiring"))

# --------------------------------------------------------------------------
#  The as-built pin map (firmware 1.3.0 defaults == what the board reports)
# --------------------------------------------------------------------------
STEPPERS = [  # joint, STEP gpio, TMC address, (MS1, MS2) straps
    ("J1", 25, 0, ("GND", "GND")),
    ("J2", 26, 1, ("+3V3", "GND")),
    ("J3", 14, 2, ("GND", "+3V3")),
]
SERVOS = [  # joint, PWM gpio, part, note
    ("J4", 18, "MG996R", "270 deg, wrist pitch"),
    ("J5", 19, "MG995", "270 deg, wrist roll"),
    ("J6", 23, "MG995", "270 deg, wrist pitch 2"),
    ("GRIP", 4, "MG995 360", "CONTINUOUS rotation, tendon spool"),
]
EN_GPIO, TMC_TX, TMC_RX = 13, 17, 16

_n = 0


def uid() -> str:
    global _n
    _n += 1
    return str(uuid.uuid5(uuid.UUID(ROOT_UUID), f"item-{_n}"))


# --------------------------------------------------------------------------
#  Library symbols
# --------------------------------------------------------------------------
def lib_block(lib: str, name: str) -> str:
    s = (KICAD_SYM / f"{lib}.kicad_sym").read_text(encoding="utf-8")
    i = s.find(f'(symbol "{name}"')
    if i < 0:
        raise SystemExit(f"{lib}:{name} not found")
    depth = 0
    for j in range(i, len(s)):
        if s[j] == "(":
            depth += 1
        elif s[j] == ")":
            depth -= 1
            if depth == 0:
                blk = s[i:j + 1]
                break
    if "(extends" in blk:
        raise SystemExit(f"{lib}:{name} extends another symbol - pick a base symbol")
    return blk.replace(f'(symbol "{name}"', f'(symbol "{lib}:{name}"', 1)


def fnt(size=1.27, hide=False, justify=None):
    j = f" (justify {justify})" if justify else ""
    h = " (hide yes)" if hide else ""
    return f"(effects (font (size {size} {size})){j}{h})"


def custom_symbol(lib_id: str, width: float, pins_spec: list, title: str) -> tuple[str, dict]:
    """A rectangle with pins on any side. pins_spec: (number, name, type, side)
    with side in L/R/T/B. Returns (sexpr, {pin_number: (x, y, angle)})."""
    short = lib_id.split(":")[1]
    side = {k: [p for p in pins_spec if p[3] == k] for k in "LRTB"}
    rows = max(len(side["L"]), len(side["R"]))
    # Pins on the top/bottom edge print their names vertically INTO the body,
    # so leave two empty rows on that edge or they run over the side pins.
    pad_t = 2 if side["T"] else 0
    pad_b = 2 if side["B"] else 0
    top = round((rows + 1 + pad_t + pad_b) * 2.54 / 2 / 1.27) * 1.27
    pins, body = {}, []
    for k, (num, pname, etype, _) in enumerate(side["L"] + side["R"]):
        left = k < len(side["L"])
        idx = k if left else k - len(side["L"])
        if num is None:
            continue
        y = top - 2.54 * (idx + 1 + pad_t)
        x = -width / 2 - 5.08 if left else width / 2 + 5.08
        pins[num] = (x, y, 0 if left else 180)
    for edge in "TB":
        n = len(side[edge])
        for k, (num, pname, etype, _) in enumerate(side[edge]):
            x = (k - (n - 1) / 2) * 5.08
            y = top + 5.08 if edge == "T" else -top - 5.08
            pins[num] = (x, y, 270 if edge == "T" else 90)
    out = [f'(symbol "{lib_id}" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)',
           f'(property "Reference" "U" (at 0 {top + 8:.2f} 0) {fnt()})',
           f'(property "Value" "{short}" (at 0 {-top - 8:.2f} 0) {fnt()})',
           f'(property "Footprint" "" (at 0 0 0) {fnt(hide=True)})',
           f'(property "Datasheet" "" (at 0 0 0) {fnt(hide=True)})',
           f'(property "Description" "{title}" (at 0 0 0) {fnt(hide=True)})',
           f'(symbol "{short}_0_1" (rectangle (start {-width / 2:.2f} {top:.2f}) (end {width / 2:.2f} {-top:.2f}) '
           f'(stroke (width 0.254) (type default)) (fill (type background))))',
           f'(symbol "{short}_1_1"']
    names = {p[0]: (p[1], p[2]) for p in pins_spec if p[0] is not None}
    for num, (x, y, ang) in pins.items():
        pname, etype = names[num]
        out.append(f'(pin {etype} line (at {x:.2f} {y:.2f} {ang}) (length 5.08) '
                   f'(name "{pname}" {fnt()}) (number "{num}" {fnt()}))')
    out.append(")")
    out.append("(embedded_fonts no))")
    return "\n".join(out), pins


P, B = "passive", "bidirectional"
ESP32_PINS = [
    ("1", "3V3", P, "T"), ("2", "GND", P, "B"),
    ("3", "5V", P, "L"), (None, "", "", "L"), ("4", "USB (CH340)", P, "L"),
    ("10", "GPIO25", B, "R"), ("11", "GPIO26", B, "R"), ("12", "GPIO14", B, "R"),
    ("13", "GPIO13", B, "R"), (None, "", "", "R"),
    ("14", "GPIO17 (TX2)", B, "R"), ("15", "GPIO16 (RX2)", B, "R"), (None, "", "", "R"),
    ("16", "GPIO18", B, "R"), ("17", "GPIO19", B, "R"), ("18", "GPIO23", B, "R"), ("19", "GPIO4", B, "R"),
]
TMC_PINS = [
    ("9", "VS", P, "T"), ("15", "VIO", P, "T"),
    ("1", "EN", P, "L"), ("4", "PDN_UART", P, "L"), ("5", "TX pad", P, "L"),
    ("7", "STEP", P, "L"), ("8", "DIR", P, "L"),
    ("12", "A1", P, "R"), ("11", "A2", P, "R"), ("13", "B1", P, "R"), ("14", "B2", P, "R"),
    ("2", "MS1", P, "B"), ("3", "MS2", P, "B"), ("6", "CLK", P, "B"), ("10", "GND", P, "B"), ("16", "GND", P, "B"),
]


# --------------------------------------------------------------------------
#  Schematic builder
# --------------------------------------------------------------------------
class Sch:
    def __init__(self):
        self.items: list[str] = []
        self.pins: dict[str, dict] = {}       # lib_id -> {num: (x, y, angle)}
        self.lib: list[str] = []
        self.custom: list[str] = []          # the project-local "arm" library
        self.refs: dict[str, int] = {}

    def use_lib(self, lib: str, name: str):
        blk = lib_block(lib, name)
        lib_id = f"{lib}:{name}"
        self.lib.append(blk)
        pins = {}
        import re
        for m in re.finditer(r'\(pin \w+ \w+\s*\(at ([-\d.]+) ([-\d.]+) (\d+)\)\s*\(length ([\d.]+)\)'
                             r'.*?\(number "([^"]+)"', blk, re.S):
            x, y, a, _ln, num = m.groups()
            pins[num] = (float(x), float(y), int(a))
        self.pins[lib_id] = pins

    def use_custom(self, lib_id, width, pins_spec, title):
        blk, pins = custom_symbol(lib_id, width, pins_spec, title)
        self.lib.append(blk)
        self.custom.append(blk.replace(f'(symbol "{lib_id}"', f'(symbol "{lib_id.split(":")[1]}"', 1))
        self.pins[lib_id] = pins

    def ref(self, prefix: str) -> str:
        self.refs[prefix] = self.refs.get(prefix, 0) + 1
        return f"{prefix}{self.refs[prefix]}"

    # placement --------------------------------------------------------------
    def place(self, lib_id, ref, value, x, y, rot=0, ref_xy=None, val_xy=None,
              hide_ref=False, hide_val=False, footprint=""):
        pins = self.pins[lib_id]
        rx, ry = ref_xy or (x, y - 6)
        vx, vy = val_xy or (x, y + 6)
        pin_lines = "".join(f'(pin "{n}" (uuid "{uid()}"))' for n in pins)
        # Field angles are stored in the symbol's rotated frame: giving them the
        # symbol's own rotation is what keeps the text horizontal on the sheet.
        fa = rot if rot in (90, 270) else 0
        self.items.append(
            f'(symbol (lib_id "{lib_id}") (at {x:.2f} {y:.2f} {rot}) (unit 1) (exclude_from_sim no) '
            f'(in_bom {"no" if ref.startswith("#") else "yes"}) (on_board yes) (dnp no) (uuid "{uid()}") '
            f'(property "Reference" "{ref}" (at {rx:.2f} {ry:.2f} {fa}) {fnt(hide=hide_ref)}) '
            f'(property "Value" "{value}" (at {vx:.2f} {vy:.2f} {fa}) {fnt(hide=hide_val)}) '
            f'(property "Footprint" "{footprint}" (at {x:.2f} {y:.2f} 0) {fnt(hide=True)}) '
            f'(property "Datasheet" "" (at {x:.2f} {y:.2f} 0) {fnt(hide=True)}) '
            f'{pin_lines} (instances (project "{NAME}" (path "/{ROOT_UUID}" (reference "{ref}") (unit 1)))))')
        return Placed(self, lib_id, x, y, rot)

    def wire(self, a, b):
        self.items.append(f'(wire (pts (xy {a[0]:.2f} {a[1]:.2f}) (xy {b[0]:.2f} {b[1]:.2f})) '
                          f'(stroke (width 0) (type default)) (uuid "{uid()}"))')

    def label(self, name, x, y, angle=0):
        just = {0: "left bottom", 180: "right bottom", 90: "left bottom", 270: "right bottom"}[angle]
        self.items.append(f'(label "{name}" (at {x:.2f} {y:.2f} {angle}) '
                          f'{fnt(justify=just)} (uuid "{uid()}"))')

    # rotation that makes a power symbol point along the stub, away from the pin
    UP_ROT = {(0, -1): 0, (0, 1): 180, (-1, 0): 90, (1, 0): 270}      # +3V3, PWR_FLAG
    DOWN_ROT = {(0, 1): 0, (0, -1): 180, (-1, 0): 270, (1, 0): 90}    # GND

    def power(self, kind, x, y, d=None):
        """GND / +3V3 symbol with its pin at (x, y), pointing along d."""
        if d is None:
            d = (0, 1) if kind == "GND" else (0, -1)
        rot = (self.DOWN_ROT if kind == "GND" else self.UP_ROT)[tuple(d)]
        vx, vy = x + d[0] * 5.5, y + d[1] * 5.0
        self.place(f"power:{kind}", self.ref("#PWR"), kind, x, y, rot,
                   ref_xy=(vx, vy + 2), val_xy=(vx, vy), hide_ref=True, hide_val=(kind == "GND"))

    def flag(self, x, y, d=(0, -1)):
        vx, vy = x + d[0] * 6, y + d[1] * 5.5
        self.place("power:PWR_FLAG", self.ref("#FLG"), "PWR_FLAG", x, y, self.UP_ROT[tuple(d)],
                   ref_xy=(vx, vy - 2), val_xy=(vx, vy), hide_ref=True, hide_val=True)

    def nc(self, x, y):
        self.items.append(f'(no_connect (at {x:.2f} {y:.2f}) (uuid "{uid()}"))')

    def text(self, s, x, y, size=1.27):
        s = s.replace('"', "'")
        self.items.append(f'(text "{s}" (exclude_from_sim no) (at {x:.2f} {y:.2f} 0) '
                          f'{fnt(size=size, justify="left top")} (uuid "{uid()}"))')

    def write(self, path: Path, title: str):
        body = "\n".join(self.items)
        libs = "\n".join(self.lib)
        path.write_text(
            f'(kicad_sch (version 20250114) (generator "eeschema") (generator_version "9.0")\n'
            f'(uuid "{ROOT_UUID}")\n(paper "A3")\n'
            f'(title_block (title "{title}") (date "2026-11-07") (rev "fw 1.3.0") '
            f'(company "Samer Haddadin") (comment 1 "As built: breadboard / point-to-point. Generated by tools/make_wiring_schematic.py") '
            f'(comment 2 "Pin numbers read from the controller (pins command), not from older docs"))\n'
            f'(lib_symbols\n{libs}\n)\n{body}\n'
            f'(sheet_instances (path "/" (page "1")))\n(embedded_fonts no)\n)\n',
            encoding="utf-8")


class Placed:
    def __init__(self, sch, lib_id, x, y, rot):
        self.sch, self.lib_id, self.x, self.y, self.rot = sch, lib_id, x, y, rot

    def pin(self, num):
        px, py, a = self.sch.pins[self.lib_id][num]
        t = math.radians(self.rot)
        rx = px * math.cos(t) - py * math.sin(t)
        ry = px * math.sin(t) + py * math.cos(t)
        # outward = opposite of the pin's body-facing direction, then rotated
        ox, oy = -math.cos(math.radians(a)), -math.sin(math.radians(a))
        dx = ox * math.cos(t) - oy * math.sin(t)
        dy = ox * math.sin(t) + oy * math.cos(t)
        return (round(self.x + rx, 2), round(self.y - ry, 2)), (round(dx), round(-dy))

    def to_label(self, num, net, length=5.08):
        (x, y), (dx, dy) = self.pin(num)
        e = (x + dx * length, y + dy * length)
        self.sch.wire((x, y), e)
        ang = {(1, 0): 0, (-1, 0): 180, (0, -1): 90, (0, 1): 270}[(dx, dy)]
        self.sch.label(net, e[0], e[1], ang)
        return e

    def to_power(self, num, kind, length=2.54):
        (x, y), (dx, dy) = self.pin(num)
        e = (x + dx * length, y + dy * length)
        self.sch.wire((x, y), e)
        self.sch.power(kind, e[0], e[1], (dx, dy))
        return e

    def nc(self, num):
        (x, y), _ = self.pin(num)
        self.sch.nc(x, y)


# --------------------------------------------------------------------------
def build() -> Sch:
    s = Sch()
    for lib, name in [("Device", "R"), ("Device", "C_Polarized"), ("Motor", "Stepper_Motor_bipolar"),
                      ("Motor", "Motor_Servo"), ("Connector_Generic", "Conn_01x02"),
                      ("power", "GND"), ("power", "+3V3"), ("power", "PWR_FLAG")]:
        s.use_lib(lib, name)
    s.use_custom("arm:ESP32_DevKit", 30.48, ESP32_PINS,
                 "ESP32-DevKit, ESP32-WROOM-32, CH340 USB-serial")
    s.use_custom("arm:TMC2209_StepStick", 25.4, TMC_PINS,
                 "TMC2209 StepStick module, UART mode")

    def flagged(placed, num, net, label=False):
        """Pin -> stub -> net, with a PWR_FLAG on a short branch off the stub end."""
        _, (dx, dy) = placed.pin(num)
        e = placed.to_label(num, net, 7.62) if label else placed.to_power(num, net, 7.62)
        fx, fy = e[0] - 5.08 * abs(dy), e[1] - 5.08 * abs(dx)
        s.wire(e, (fx, fy))
        s.flag(fx, fy, (0, -1) if dx else (-1, 0))

    # ---------------- power inputs (left column) ----------------
    s.text("POWER", 22.86, 30.48, 1.8)
    jm = s.place("Connector_Generic:Conn_01x02", "J1", "Motor PSU 12-24 V", 30.48, 48.26,
                 ref_xy=(30.48, 42), val_xy=(30.48, 55))
    flagged(jm, "1", "VMOT", label=True)
    jm.to_power("2", "GND", 7.62)
    js = s.place("Connector_Generic:Conn_01x02", "J2", "Servo PSU 6 V, 5 A+", 30.48, 76.2,
                 ref_xy=(30.48, 70), val_xy=(30.48, 83))
    flagged(js, "1", "SERVO_6V", label=True)
    js.to_power("2", "GND", 7.62)
    cb = s.place("Device:C_Polarized", s.ref("C"), "1000uF 10V", 76.2, 76.2,
                 ref_xy=(80, 74.5), val_xy=(80, 78.5))
    cb.to_label("1", "SERVO_6V", 2.54)
    cb.to_power("2", "GND", 2.54)
    s.text("bulk cap on the servo rail, near the wrist servos", 66, 86, 1.0)

    # ---------------- ESP32 ----------------
    s.text("CONTROLLER", 22.86, 104.14, 1.8)
    esp = s.place("arm:ESP32_DevKit", "U1", "ESP32-DevKit (WROOM-32, CH340)", 55.88, 144.78,
                  ref_xy=(36, 120), val_xy=(84, 188))
    flagged(esp, "1", "+3V3")
    flagged(esp, "2", "GND")
    esp.nc("3")
    esp.nc("4")
    s.text("USB to the PC: power + serial 115200 baud", 22.86, 192, 1.0)
    nets = {"10": "J1_STEP", "11": "J2_STEP", "12": "J3_STEP", "13": "DRV_EN",
            "14": "TMC_TX", "15": "TMC_UART", "16": "J4_PWM", "17": "J5_PWM",
            "18": "J6_PWM", "19": "GRIP_PWM"}
    for num, net in nets.items():
        esp.to_label(num, net)

    # UART: one 1 k in the TX line, RX straight onto the bus (half duplex)
    ru = s.place("Device:R", s.ref("R"), "1k", 119.38, 154.94, rot=90,
                 ref_xy=(119.38, 150.5), val_xy=(119.38, 159.5))
    ru.to_label("1", "TMC_TX", 5.08)
    ru.to_label("2", "TMC_UART", 5.08)
    s.text("half-duplex UART: the only 1k is in TX", 104, 163, 1.0)
    # EN pull-up keeps every driver OFF while the ESP32 boots or reflashes
    re_ = s.place("Device:R", s.ref("R"), "10k", 119.38, 127, rot=0,
                  ref_xy=(123.5, 125.5), val_xy=(123.5, 129))
    re_.to_power("1", "+3V3", 2.54)
    re_.to_label("2", "DRV_EN", 2.54)
    s.text("EN pull-up: drivers stay limp during boot and flashing", 124, 120, 1.0)

    # ---------------- drivers + motors ----------------
    s.text("STEPPER JOINTS  (TMC2209 on one UART bus, DIR unused)", 152.4, 30.48, 1.8)
    for k, (jn, step, addr, (ms1, ms2)) in enumerate(STEPPERS):
        x, y = 185.42 + k * 78.74, 66.04
        d = s.place("arm:TMC2209_StepStick", f"U{2 + k}", f"TMC2209 {jn}  addr {addr}", x, y,
                    ref_xy=(x - 22, y - 16), val_xy=(x, y + 30))
        d.to_label("9", "VMOT", 5.08)
        d.to_power("15", "+3V3", 5.08)
        d.to_label("1", "DRV_EN")
        d.to_label("4", "TMC_UART")
        d.nc("5")
        d.to_label("7", f"{jn}_STEP")
        d.nc("8")
        d.to_power("2", ms1, 5.08)
        d.to_power("3", ms2, 5.08)
        d.to_power("6", "GND", 5.08)
        d.to_power("10", "GND", 5.08)
        d.to_power("16", "GND", 5.08)
        for num, coil in (("12", "A1"), ("11", "A2"), ("13", "B1"), ("14", "B2")):
            d.to_label(num, f"{jn}_{coil}", 7.62)
        c = s.place("Device:C_Polarized", s.ref("C"), "100uF 35V", x + 33.02, y - 13.97,
                    ref_xy=(x + 37, y - 15.5), val_xy=(x + 37, y - 12))
        c.to_label("1", "VMOT", 2.54)
        c.to_power("2", "GND", 2.54)
        m = s.place("Motor:Stepper_Motor_bipolar", f"M{1 + k}", f"17HS4401S ({jn})", x + 7.62, y + 58.42,
                    ref_xy=(x + 17, y + 54), val_xy=(x + 7.62, y + 68))
        m.to_label("1", f"{jn}_A1", 2.54)
        m.to_label("2", f"{jn}_A2", 2.54)
        m.to_label("3", f"{jn}_B1", 2.54)
        m.to_label("4", f"{jn}_B2", 2.54)

    # ---------------- servos ----------------
    s.text("SERVO JOINTS + GRIPPER  (LEDC PWM 50 Hz, own 6 V supply)", 152.4, 162.56, 1.8)
    for k, (jn, gpio, part, note) in enumerate(SERVOS):
        x, y = 193.04 + k * 58.42, 182.88
        sv = s.place("Motor:Motor_Servo", f"M{4 + k}", f"{part} ({jn})", x, y,
                     ref_xy=(x + 2, y - 7), val_xy=(x + 2, y + 7.5))
        sv.to_label("1", f"{jn}_PWM", 7.62)
        sv.to_label("2", "SERVO_6V", 7.62)
        sv.to_power("3", "GND", 7.62)
        s.text(note, x - 16, y + 11, 1.0)

    # ---------------- notes ----------------
    notes = [
        "WIRING NOTES  (what bit us, roughly in order of how much time it cost)",
        "1. TMC2209 UART needs VS. VIO only powers the I/O ring; with no motor supply the driver never answers.",
        "2. R7 on each StepStick bridged pad 2-3 so PDN_UART reaches header pin 4. Check: pin 4 to GND reads ~20k.",
        "3. One 1k total, in the ESP32 TX line (GPIO17). RX (GPIO16) goes straight to the bus. One wire, half duplex.",
        "4. Bus address latched from MS1/MS2 at power-up: J1=0 (GND,GND), J2=1 (3V3,GND), J3=2 (GND,3V3). Max 4 per bus.",
        "5. DIR is NOT wired: the DIR inputs on these modules are damaged. Direction = GCONF.shaft over UART (fw 1.2+).",
        "6. EN is active low and shared. The 10k pull-up keeps the motors limp while the ESP32 boots or reflashes.",
        "7. CLK to GND = internal oscillator. Microstepping is set over UART (MRES); MS1/MS2 are address pins only.",
        "8. Never plug or unplug a motor while VMOT is live - the back-EMF spike kills the driver.",
        "9. Servos never take power from the dev board. Common GND between ESP32, motor PSU and servo PSU is mandatory.",
        "10. GRIP is a continuous-rotation servo: pulse width = speed, ~1500 us = stop. See docs/calibration.md.",
        "11. 17HS4401S coils: usually black/green = A, red/blue = B. Confirm with a meter (a few ohms within a coil).",
    ]
    for i, line in enumerate(notes):
        s.text(line, 22.86, 203.2 + i * 4.3, 1.5 if i == 0 else 1.15)
    return s


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    s = build()
    s.write(OUT / f"{NAME}.kicad_sch",
            "Robotic arm - wiring as built")
    pro = OUT / f"{NAME}.kicad_pro"
    if not pro.exists():
        pro.write_text('{\n  "meta": {"filename": "%s.kicad_pro", "version": 3},\n'
                       '  "sheets": [["%s", "Root"]]\n}\n' % (NAME, ROOT_UUID), encoding="utf-8")
    # The two module blocks as a real, editable project library.
    (OUT / "arm.kicad_sym").write_text(
        '(kicad_symbol_lib (version 20241209) (generator "make_wiring_schematic") '
        '(generator_version "9.0")\n' + "\n".join(s.custom) + "\n)\n", encoding="utf-8")
    (OUT / "sym-lib-table").write_text(
        '(sym_lib_table\n  (version 7)\n  (lib (name "arm")(type "KiCad")'
        '(uri "${KIPRJMOD}/arm.kicad_sym")(options "")(descr "Robotic arm module blocks"))\n)\n',
        encoding="utf-8")
    print("wrote", OUT / f"{NAME}.kicad_sch")


if __name__ == "__main__":
    main()

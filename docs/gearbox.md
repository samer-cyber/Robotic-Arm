# Gearbox notes

## How the arm uses its gearboxes

Each geared joint is split between two links. The **carrier** (motor plate +
output screws) prints as part of one link, and the **drum** (roller ring) as part
of the next. Read off the Fusion model (`Full arm (no PCB) v25`):

| Joint | Carrier + motor live in | Drum lives in | Rollers | Bearings |
|---|---|---|---|---|
| J1 base | Body 1 (base) | Body 2 (turret) | 16 | one |
| J2 shoulder | Body 2 (turret) | Body 3 (upper arm) | 21 | one |
| J3 elbow | Body 4 (forearm) | Body 3 (upper arm) | 16 | two |

So the upper arm is just two drums and a beam with no motor on it, and the
elbow motor rides on the forearm. J2→J3 and J3→J4 are both 130 mm.

## The ratio is the roller count

With the screws fixed to the motor, the discs cannot rotate; they only orbit.
Each eccentric turn walks the drum by one roller pitch:

```
classic  (ring fixed, screws = output):  ratio = lobes   = N - 1, output reversed
inverted (screws fixed, drum = output):  ratio = rollers = N,     output same way
```

The firmware used 15/20/15 (the lobe counts) for boxes that are 16/21/16, so
every stepper joint fell 5-6 % short of its target. A 90° command landed near
84-86°. Firmware 1.3.0 fixes the defaults; [calibration.md](calibration.md)
fixes the copy saved in flash.

## J2's second disc interferes

The shoulder box has 21 rollers, an odd number. Its disc B, drawn by the v6
generator as disc A turned 180°, overlaps the roller ring by **185 mm³** in the
CAD (checked with a boolean intersection on `20:1 Gearbox v8`). The 16-roller
boxes are clean. The correct second disc is turned back by half a lobe
(π / lobes) relative to the ring. That angle equals 180° only for even roller
counts.

Expect J2 to run rougher and need more current than the others (1200 mA was
where it stopped losing steps). Reprint disc B from the v7 generator, or in
Fusion rotate the existing disc B by 9° about its own centre (half of the
18° lobe pitch), then check it against the ring with Inspect > Interference.

## One bearing or two

The one-bearing drum hangs off a single 6808 on the motor side, and its far
end is free. Any radial load tips it, which is the wobble the arm had. The
two-bearing version clamps the drum between two 6808s, held by the far carrier
and the six screws as one cage. It costs a second bearing and 2.5 mm of length.
J3 already uses it; J1 and J2 would benefit most.

## Generator

[`hardware/gearbox-generator`](../hardware/gearbox-generator) builds all of
this from a dialog, for any ratio, bearing set and NEMA size, plus an optional
link to the next joint.

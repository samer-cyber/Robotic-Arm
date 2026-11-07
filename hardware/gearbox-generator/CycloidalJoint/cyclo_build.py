"""Cycloidal joint - Fusion geometry. All decisions live in cyclo_core.plan();
this file only draws them. Everything goes into ONE new component per joint,
so a design can hold several joints and you move them as units.

Bodies (print the first five, never the REF ones):
    Drum (output)          roller ring + bearing seats. Join your link to it.
    Carrier - motor side   motor bolts to it; the output screws thread into it
    Carrier - far side     two-bearing version only; screw heads sit on it
    Disc A, Disc B         cycloidal discs, 180 deg apart
    Eccentric              two offset journals on the motor's D-shaft
    Link ...               optional arm link, join it to the drum (and to the
                           next joint's drum at x = link length)
"""
from __future__ import annotations

import math

import adsk.core
import adsk.fusion

from cyclo_core import Plan, disc_profile

MM = 0.1   # Fusion's internal unit is the centimetre

# Live print tolerances, shared by every joint in the design: they describe
# your printer, not the gearbox. Modify > Change Parameters and it rebuilds.
TOLERANCES = [
    ("cj_tol_disc_bore",  0.10, "Disc bore over the cam bearing OD"),
    ("cj_tol_journal",    0.04, "Eccentric journal under the cam bearing bore"),
    ("cj_tol_big_seat",   0.08, "Drum bearing seat over the big bearing OD"),
    ("cj_tol_carrier",    0.05, "Carrier journal under the big bearing bore"),
    ("cj_tol_motor_bore", 0.25, "Eccentric bore over the motor shaft"),
    ("cj_tol_out_hole",   0.25, "Disc output holes, on top of screw + 2E. Print one, measure, adjust"),
]


class Builder:
    def __init__(self, design: adsk.fusion.Design, comp: adsk.fusion.Component):
        self.des, self.c = design, comp
        self.ef = comp.features.extrudeFeatures

    # ----------------------------------------------------------- primitives --
    def _sketch(self):
        sk = self.c.sketches.add(self.c.xYConstructionPlane)
        sk.isComputeDeferred = True
        return sk

    def _profiles(self, sk, keep=None):
        sk.isComputeDeferred = False
        coll = adsk.core.ObjectCollection.create()
        for pr in sk.profiles:
            if keep is None or keep(pr):
                coll.add(pr)
        return coll

    def extrude(self, prof, z0, dz, op, target=None, name=None):
        inp = self.ef.createInput(prof, op)
        inp.startExtent = adsk.fusion.OffsetStartDefinition.create(
            adsk.core.ValueInput.createByReal(z0 * MM))
        inp.setOneSideExtent(
            adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(dz * MM)),
            adsk.fusion.ExtentDirections.PositiveExtentDirection)
        if target is not None:
            inp.participantBodies = [target]
        f = self.ef.add(inp)
        if name and f.bodies.count:
            f.bodies.item(0).name = name
        return f

    def circles(self, centers, r, z0, dz, op, target=None, name=None):
        sk = self._sketch()
        for (x, y) in centers:
            sk.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(x * MM, y * MM, 0), r * MM)
        f = self.extrude(self._profiles(sk), z0, dz, op, target, name)
        return f.bodies.item(0) if op == NB else f

    def live_circles(self, centers, expr, z0, dz, op, target=None):
        """Circles whose DIAMETER follows a user parameter expression."""
        sk = self.c.sketches.add(self.c.xYConstructionPlane)
        for (x, y) in centers:
            c = sk.sketchCurves.sketchCircles.addByCenterRadius(
                adsk.core.Point3D.create(x * MM, y * MM, 0), 1.0 * MM)
            d = sk.sketchDimensions.addDiameterDimension(
                c, adsk.core.Point3D.create((x + 2) * MM, (y + 2) * MM, 0))
            d.parameter.expression = expr
        return self.extrude(self._profiles(sk), z0, dz, op, target)

    def annulus(self, ro, ri, z0, dz, op, target=None, name=None, cx=0.0, cy=0.0):
        sk = self._sketch()
        P = adsk.core.Point3D.create
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(cx * MM, cy * MM, 0), ro * MM)
        sk.sketchCurves.sketchCircles.addByCenterRadius(P(cx * MM, cy * MM, 0), ri * MM)
        prof = self._profiles(sk, keep=lambda pr: pr.profileLoops.count == 2)
        f = self.extrude(prof, z0, dz, op, target, name)
        return f.bodies.item(0) if op == NB else f

    def rect(self, x0, y0, x1, y1, z0, dz, op, target=None):
        sk = self._sketch()
        sk.sketchCurves.sketchLines.addTwoPointRectangle(
            adsk.core.Point3D.create(x0 * MM, y0 * MM, 0), adsk.core.Point3D.create(x1 * MM, y1 * MM, 0))
        return self.extrude(self._profiles(sk), z0, dz, op, target)

    def spline(self, pts, z0, dz, name):
        sk = self._sketch()
        coll = adsk.core.ObjectCollection.create()
        for (x, y) in pts:
            coll.add(adsk.core.Point3D.create(x * MM, y * MM, 0))
        sk.sketchCurves.sketchFittedSplines.add(coll)
        sk.isComputeDeferred = False
        return self.extrude(sk.profiles.item(0), z0, dz, NB, name=name).bodies.item(0)

    def d_bore(self, shaft_d, flat, z0, dz, target):
        """Motor D-profile, diameter live via cj_tol_motor_bore."""
        sk = self.c.sketches.add(self.c.xYConstructionPlane)
        P = adsk.core.Point3D.create
        c = sk.sketchCurves.sketchCircles.addByCenterRadius(P(0, 0, 0), shaft_d / 2 * MM)
        d = sk.sketchDimensions.addDiameterDimension(c, P(3 * MM, 3 * MM, 0))
        d.parameter.expression = "%g mm + cj_tol_motor_bore" % shaft_d
        fx = flat - shaft_d / 2.0
        sk.sketchCurves.sketchLines.addByTwoPoints(P(fx * MM, -shaft_d * MM, 0), P(fx * MM, shaft_d * MM, 0))
        best, prof = -1.0, None
        for pr in sk.profiles:
            a = pr.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).area
            if a > best:
                best, prof = a, pr
        return self.extrude(prof, z0, dz, CU, target)

    def fillet_rims(self, body, r):
        """Round the outer rim of a disc's two flat faces. Cosmetic: skipped
        quietly if Fusion refuses."""
        edges = adsk.core.ObjectCollection.create()
        for face in body.faces:
            if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                continue
            for loop in face.loops:
                if loop.isOuter:
                    for e in loop.edges:
                        edges.add(e)
        try:
            ff = self.c.features.filletFeatures
            fi = ff.createInput()
            fi.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(r * MM), True)
            ff.add(fi)
            return True
        except Exception:
            return False


NB = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JO = adsk.fusion.FeatureOperations.JoinFeatureOperation
CU = adsk.fusion.FeatureOperations.CutFeatureOperation


def ensure_parameters(des: adsk.fusion.Design):
    for name, val, comment in TOLERANCES:
        if not des.userParameters.itemByName(name):
            des.userParameters.add(name, adsk.core.ValueInput.createByReal(val * MM), "mm", comment)


def build(des: adsk.fusion.Design, p: Plan) -> adsk.fusion.Occurrence:
    if not p.ok:
        raise ValueError("plan has errors:\n" + "\n".join(p.errors))
    ensure_parameters(des)
    s, z = p.spec, p.z
    N, R, rp, E = p.N, p.R, p.rp, p.E
    d_b, D_b, w_b = p.big
    d_c, D_c, w_c = p.cam
    m, scr = p.motor, p.scr

    occ = des.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = "Cycloidal joint %d-1 (%s)" % (N, "2 bearings" if s.two_bearings else "1 bearing")
    b = Builder(des, comp)
    notes = []

    # ------------------------------------------------------------------ drum --
    z0, z1 = z["drum0"], z["drum1"]
    drum = b.circles([(0, 0)], R + s.wall, z0, z1 - z0, NB, name="Drum (output - join your link here)")
    inner0 = z["far_bearing1"] if s.two_bearings else z["floor1"]
    if s.two_bearings:
        b.live_circles([(0, 0)], "%g mm + cj_tol_big_seat" % D_b, z["far_bearing0"], w_b, CU, drum)
    b.circles([(0, 0)], R, inner0, z["near_bearing0"] - inner0, CU, drum)
    rollers = [(R * math.cos(2 * math.pi * k / N), R * math.sin(2 * math.pi * k / N)) for k in range(N)]
    b.circles(rollers, rp, inner0, z["near_bearing0"] - inner0, JO, drum)
    b.live_circles([(0, 0)], "%g mm + cj_tol_big_seat" % D_b, z["near_bearing0"], w_b, CU, drum)
    # the rollers sit ON the ring circle, so they poke out by rp - wall: trim
    b.annulus(R + s.wall + 20, R + s.wall, z0, z1 - z0, CU, drum)

    # ---------------------------------------------------- motor-side carrier --
    lip_r = d_b / 2.0 + min(1.5, (D_b - d_b) / 6.0)
    nc = b.live_circles([(0, 0)], "%g mm - cj_tol_carrier" % d_b, z["near_bearing0"], w_b, NB).bodies.item(0)
    nc.name = "Carrier - motor side"
    b.circles([(0, 0)], lip_r, z["near_bearing1"], z["lip1"] - z["near_bearing1"], JO, nc)
    b.circles([(0, 0)], p.flange_od / 2.0, z["lip1"], z["motor_face"] - z["lip1"], JO, nc)
    b.circles([(0, 0)], (m["shaft_d"] + 1.0) / 2.0, z["near_bearing0"],
              z["motor_face"] - z["near_bearing0"], CU, nc)
    b.circles([(0, 0)], (m["boss_d"] + 0.4) / 2.0, z["motor_face"] - m["boss_h"] - 0.2,
              m["boss_h"] + 0.2, CU, nc)
    hs = m["bolt_spacing"] / 2.0
    bolts = [(hs, hs), (-hs, hs), (-hs, -hs), (hs, -hs)]
    b.circles(bolts, m["bolt_clear"] / 2.0, z["near_bearing1"], z["motor_face"] - z["near_bearing1"], CU, nc)
    b.circles(bolts, m["cbore_d"] / 2.0, z["near_bearing1"],
              (z["lip1"] - z["near_bearing1"]) + m["head_h"] + 0.3, CU, nc)
    pins = [(p.screw_r * math.cos(2 * math.pi * j / s.screw_count),
             p.screw_r * math.sin(2 * math.pi * j / s.screw_count)) for j in range(s.screw_count)]
    b.circles(pins, scr["tap"] / 2.0, z["near_bearing0"], z["motor_face"] - 1.0 - z["near_bearing0"], CU, nc)

    # ------------------------------------------------------ far-side carrier --
    if s.two_bearings:
        fc = b.circles([(0, 0)], lip_r, z["far_plate0"], -z["far_plate0"], NB, name="Carrier - far side")
        b.live_circles([(0, 0)], "%g mm - cj_tol_carrier" % d_b, z["far_bearing0"], w_b, JO, fc)
        b.circles(pins, scr["clear"] / 2.0, z["far_plate0"], w_b - z["far_plate0"], CU, fc)

    # ----------------------------------------------------------------- discs --
    for phase, key, label in ((0.0, "discA", "Disc A"), (math.pi, "discB", "Disc B")):
        zz = z[key + "0"]
        disc = b.spline(disc_profile(p, phase), zz, w_c, label)
        cx = E * math.cos(phase)
        b.live_circles([(cx, 0)], "%g mm + cj_tol_disc_bore" % D_c, zz, w_c, CU, disc)
        holes = [(cx + x, y) for (x, y) in pins]
        b.live_circles(holes, "%g mm + cj_tol_out_hole" % p.hole_d, zz, w_c, CU, disc)
        if not any(b.fillet_rims(disc, r) for r in (0.6, 0.4, 0.25)):
            notes.append("%s: rim fillet skipped (cosmetic)." % label)

    # ------------------------------------------------------------- eccentric --
    zmid = z["discA1"] + s.disc_gap / 2.0
    # Just the two offset journals, nothing concentric beyond them: each cam
    # bearing has to slide on from its own end, and anything round the shaft
    # axis would stand proud of the offset journal and block it.
    ecc = b.live_circles([(E, 0)], "%g mm - cj_tol_journal" % d_c, z["discA0"],
                         zmid - z["discA0"], NB).bodies.item(0)
    ecc.name = "Eccentric"
    b.live_circles([(-E, 0)], "%g mm - cj_tol_journal" % d_c, zmid, z["discB1"] - zmid, JO, ecc)
    b.d_bore(m["shaft_d"], m["flat"], z["discA0"], z["discB1"] - z["discA0"], ecc)

    # ------------------------------------------------------------------ link --
    if s.add_link:
        L, ro = s.link_length, p.drum_od / 2.0 + s.link_skin
        ri = p.drum_od / 2.0 - s.wall / 2.0           # overlaps half the drum wall
        link = b.circles([(0, 0)], ro, z0, z1 - z0, NB, name="Link %g mm (join to both drums)" % L)
        b.rect(0, -ro, L, ro, z0, z1 - z0, JO, link)
        b.circles([(L, 0)], ro, z0, z1 - z0, JO, link)
        b.circles([(0, 0), (L, 0)], ri, z0, z1 - z0, CU, link)
        if s.link_hollow and L - 2 * ro > 12:
            b.rect(ro + 2, -(ro - 4), L - ro - 2, ro - 4, z0, z1 - z0, CU, link)
        notes.append("Link: generate the next joint and move it to x = %g mm, then Combine > Join "
                     "the link with both drums." % L)

    # ------------------------------------------------------ reference parts --
    if s.reference_hardware:
        ref = []
        ref.append(b.annulus(D_c / 2, d_c / 2, z["discA0"], w_c, NB, name="REF cam bearing A", cx=E))
        ref.append(b.annulus(D_c / 2, d_c / 2, z["discB0"], w_c, NB, name="REF cam bearing B", cx=-E))
        ref.append(b.annulus(D_b / 2, d_b / 2, z["near_bearing0"], w_b, NB, name="REF big bearing (motor side)"))
        if s.two_bearings:
            ref.append(b.annulus(D_b / 2, d_b / 2, z["far_bearing0"], w_b, NB, name="REF big bearing (far side)"))
            head0 = z["far_plate0"] - scr["head_h"]
        else:
            head0 = z["heads0"]
        for j, (x, y) in enumerate(pins):
            sc = b.circles([(x, y)], scr["head_d"] / 2, head0, scr["head_h"], NB, name="REF screw %d" % (j + 1))
            b.circles([(x, y)], scr["d"] / 2, head0 + scr["head_h"], p.screw_buy, JO, sc)
            ref.append(sc)
        mf = z["motor_face"]
        mot = b.rect(-m["body"] / 2, -m["body"] / 2, m["body"] / 2, m["body"] / 2, mf, m["length"], NB)
        mot = mot.bodies.item(0)
        mot.name = "REF motor"
        b.circles([(0, 0)], m["boss_d"] / 2, mf - m["boss_h"], m["boss_h"], JO, mot)
        shaft_top = mf - m["boss_h"]
        dcut_lo = shaft_top - m["round_len"] - m["dcut_len"]
        b.circles([(0, 0)], m["shaft_d"] / 2, dcut_lo, m["round_len"] + m["dcut_len"], JO, mot)
        fx = m["flat"] - m["shaft_d"] / 2.0          # the D-flat, so fits check exactly
        b.rect(fx, -m["shaft_d"], m["shaft_d"], m["shaft_d"], dcut_lo, m["dcut_len"], CU, mot)
        ref.append(mot)
        try:   # tint the reference parts so nobody prints them by accident
            lib = adsk.core.Application.get().materialLibraries.itemByName("Fusion Appearance Library")
            look = lib.appearances.itemByName("Steel - Satin") if lib else None
            for rb in ref:
                if look:
                    rb.appearance = look
        except Exception:
            pass

    return occ, notes

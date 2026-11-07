#Author-Samer
#Description-Cycloidal gearbox generator - SCREWS / HEX-HEAD variant - v6 (derived axial stack, 5 live tolerances)
#
# ============================================================================
#  WHAT THIS BUILDS
#  Housing + 2 cycloidal discs + eccentric shaft. Output torque is taken through
#  M4 hex socket-cap SCREWS: disc A rides on the screw HEAD (sunk in a pocket),
#  disc B rides on the smooth SHANK. Neither disc is clamped.
#
#  HOW TO RUN: new empty Design -> Shift+S -> Scripts -> "+" -> Create -> Python
#  -> paste -> save -> Run.
#
#  THE AXIAL STACK IS NOW DERIVED, NOT TYPED.  You set the back wall; everything
#  downstream falls out of the real motor shaft geometry. See section 4.
#
#  FIVE LIVE TOLERANCES (Modify -> Change Parameters, model rebuilds):
#      tol_disc_bore    disc centre bore          CAM_OD +
#      tol_journal      eccentric journals        CAM_ID -
#      tol_big_seat     big output bearing seat   BIG_OD +
#      tol_motor_bore   motor D-shaft bore        MOTOR_SHAFT_D +
#      tol_out_hole     output holes + head pocket
#  All five are INDEPENDENT - no hidden coupling. Tune each fit on its own.
#
#  NOT live: CLEAR (profile clearance). It feeds every point of the disc spline,
#  so it cannot be a Fusion parameter without rebuilding the curve - re-run the
#  script to change it. This is a real limitation, not an oversight.
# ============================================================================

import adsk.core, adsk.fusion, traceback, math

# ============================ 1. FROM THE SPREADSHEET ========================
#  These are exactly the cells in the calculator's copy block.
RATIO         = 15      # reduction i.  rollers = i+1, lobes = i
R             = 26.0    # mm  roller circle radius (= big bearing OD / 2)
PIN_R         = 3.06    # mm  roller RADIUS. Round ONLY for real steel dowel pins
ECC           = 1.02    # mm  eccentricity E
CLEAR         = -0.08    # mm  profile clearance (baked - see header)
CAM_ID        = 8.0     # mm  cam bearing inner dia  -> eccentric journal
CAM_OD        = 22.0    # mm  cam bearing outer dia  -> disc bore
CAM_W         = 7.0     # mm  cam bearing width      -> disc thickness
BIG_OD        = 52.0    # mm  large output bearing outer dia
BIG_W         = 7.0     # mm  large output bearing width
OUT_SCREW_N   = 6       #     output screws - MUST BE EVEN
OUT_SCREW_R   = 16.5   # mm  output screw circle radius
WALL          = 3.0     # mm  outer wall / seat ring radial thickness
MOTOR_SHAFT_D = 5.0     # mm  motor shaft diameter
MOTOR_BOSS_D  = 22.0    # mm  motor front boss diameter
FLAT_TO_OPP   = 4.5     # mm  flat-to-opposite-side of the D-shaft

# ============================ 2. TOLERANCES (all live) =======================
TOL_DISC_BORE  = 0.10   # mm  disc centre bore   (CAM_OD + this)
TOL_JOURNAL    = 0.04   # mm  eccentric journals (CAM_ID - this)
TOL_BIG_SEAT   = 0.09   # mm  big bearing seat   (BIG_OD + this)
TOL_MOTOR_BORE = 0.25   # mm  motor D-shaft bore (MOTOR_SHAFT_D + this)
TOL_OUT_HOLE   = 0.3   # mm  PRINT allowance on every output hole, ON TOP of
                        #     the geometric  pin + 2E.  Zero here = guaranteed
                        #     binding if the printer runs undersize.
                        #     Set properly: print one disc, caliper a hole,
                        #     TOL_OUT_HOLE = (nominal - measured) + 0.05.

# ============================ 3. THE OUTPUT SCREW ============================
SCREW_SHANK_D = 3.90    # mm  smooth shank diameter   (measured)
SCREW_HEAD_D  = 6.88    # mm  head diameter           (measured)
SCREW_HEAD_H  = 3.84    # mm  head height             (measured)
SCREW_SHANK_L = 9.50    # mm  SMOOTH length below the head

HEAD_POCKET_DEPTH = 4.0 # mm  pocket in disc A for the head. TYPE IT DIRECTLY.
                        #     Head is 3.84, so 4.0 leaves 0.16 - almost flush.
                        #     The script reports whether the head bottoms out
                        #     (= clamping risk) and what depth would avoid it.
WEB_MIN       = 1.50    # mm  minimum radial web beside an output hole (warns)

# --- reference hardware, VISUAL ONLY. None of this changes a printed part. ---
BIG_ID        = 40.0    # mm  big bearing BORE. Not derivable from OD+width, so it
                        #     has to be stated: 6808 = 40 x 52 x 7.
HEX_AF        = 3.0     # mm  hex key across flats (M4 socket cap = 3 mm)
HEX_DEPTH     = 2.0     # mm  how deep the key socket goes into the head
SCREW_THREAD_D = 4.0    # mm  M4 nominal thread OD, drawn above the shank
SCREW_THREAD_L = 10.5   # mm  length of threaded portion to draw (M4x20 => 20 - 9.5)

# ============================ 4. THE AXIAL STACK =============================
#  Only BACK_WALL is a free choice. Everything else is derived from the real
#  motor shaft, so the numbers cannot drift out of sync.
#
#      z = 0 is the housing's back OUTER face (it sits on the arm-body wall)
#
#      motor face ---- MOTOR_STANDOFF ---- | housing back face (z=0)
#      shaft:  [ boss 2 ][ round 3 ][ ------- D-cut 18 ------- ]
#
BACK_WALL     = 4.0     # mm  <-- THE ONE YOU TUNE. Must clear the M3 counterbore.
BACK_GAP      = 2.0     # mm  FIXED. Cavity floor -> disc A. Safety gap in case a
                        #     screw tip protrudes. Deliberately not adjustable.
DISC_GAP      = 0.5     # mm  disc A -> disc B
FRONT_GAP     = 0.5     # mm  disc B -> output flange
SPACER_D      = 7.08    # mm  eccentric spacer cylinder diameter
#  SPACER_L is DERIVED so the spacer bottom lands exactly where the D-cut starts.

MOTOR_STANDOFF = 3.5    # mm  arm-body wall between motor face and housing
SHAFT_BOSS_L   = 2.0    # mm  large boss at the motor face
SHAFT_ROUND_L  = 3.0    # mm  smooth round section after it
SHAFT_D_LEN    = 18.0   # mm  the D-cut section
MIN_ENGAGE     = 10.0   # mm  warn below this much eccentric-on-D-cut overlap

# --- M3 motor mounting ---
MOUNT_SPACING = 31.0    # mm  NEMA 17 bolt spacing
M3_CLEAR_D    = 3.5     # mm
CBORE_D       = 6.5     # mm
CBORE_DEPTH   = 2.0     # mm  2.0 = flush M3 socket cap head
CBORE_FLOOR_MIN = 1.5   # mm  wall that must remain under the head

# ============================ 5. BUILD OPTIONS ===============================
DISC_FILLET_R = 0.75    # mm  rim fillet, top and bottom. 0 = skip
MAKE_HOUSING  = True
MAKE_DISCS    = True
MAKE_ECC      = True
MAKE_HARDWARE = True    # bearings + screws as REF bodies (never print these)
TWO_DISCS     = True
PTS_PER_LOBE  = 16
# =============================================================================

MM = 0.1


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        des = adsk.fusion.Design.cast(app.activeProduct)
        if not des:
            ui.messageBox('Open a Design first, then run this script.')
            return
        root = des.rootComponent

        # -------------------- derived geometry --------------------
        N, Z = RATIO + 1, RATIO
        Rre = PIN_R + CLEAR
        lam = ECC * N / R
        rmin = R - Rre - ECC
        rmax = R - Rre + ECC
        t = CAM_W

        # where the motor's D-cut lives, in housing coordinates
        z_dsh0 = SHAFT_BOSS_L + SHAFT_ROUND_L - MOTOR_STANDOFF
        z_dsh1 = z_dsh0 + SHAFT_D_LEN

        # axial stack
        zA0 = BACK_WALL + BACK_GAP
        zA1 = zA0 + t
        zB0 = zA1 + DISC_GAP
        zB1 = zB0 + t
        H_cav = BACK_GAP + t + DISC_GAP + t + FRONT_GAP
        z_cav0 = BACK_WALL
        z_cav1 = BACK_WALL + H_cav          # output flange underside
        z_seat1 = z_cav1 + BIG_W
        zjA0, zjA1 = zA0, zA1 + DISC_GAP
        zjB0, zjB1 = zA1, zB1

        # DERIVED: spacer bottom lands exactly at the start of the D-cut
        SPACER_L = zA0 - z_dsh0
        zsp0 = zA0 - SPACER_L               # == z_dsh0 by construction
        ecc_len = zjB1 - zsp0
        engage = max(0.0, min(zjB1, z_dsh1) - max(zsp0, z_dsh0))

        # the screw, seated head-down from the motor side
        head_top_ifseated = zA0 + HEAD_POCKET_DEPTH      # head bottoms on pocket floor
        shank_top_ifseated = head_top_ifseated + SCREW_SHANK_L
        pocket_for_no_clamp = (z_cav1 - SCREW_SHANK_L) - zA0   # depth that lets the
                                                               # thread shoulder reach
                                                               # the flange first
        shank_needed_for_pocket = z_cav1 - (zA0 + HEAD_POCKET_DEPTH)

        # output holes
        Dh_shank = SCREW_SHANK_D + 2.0 * ECC + TOL_OUT_HOLE
        Dh_head = SCREW_HEAD_D + 2.0 * ECC + TOL_OUT_HOLE
        half_span = (rmin - CAM_OD / 2.0) / 2.0
        web_shank = half_span - Dh_shank / 2.0
        web_head = half_span - Dh_head / 2.0

        # -------------------- validation --------------------
        fail, warn, info = [], [], []
        if lam >= 1.0:
            fail.append('lambda = E*N/R = %.3f must be < 1.' % lam)
        if PIN_R <= 2.0 * ECC:
            fail.append('PIN_R %.2f must exceed 2*ECC = %.2f.' % (PIN_R, 2 * ECC))
        if TWO_DISCS and OUT_SCREW_N % 2 != 0:
            fail.append('OUT_SCREW_N must be EVEN.')
        if CAM_OD / 2.0 >= rmin - 1.5:
            fail.append('Cam bearing OD too large for this disc.')
        if FLAT_TO_OPP >= MOTOR_SHAFT_D:
            fail.append('FLAT_TO_OPP must be < MOTOR_SHAFT_D.')
        if BACK_WALL - CBORE_DEPTH < CBORE_FLOOR_MIN:
            fail.append('BACK_WALL %.1f - counterbore %.1f leaves %.1f mm under the M3 head '
                        '(min %.1f). Raise BACK_WALL.'
                        % (BACK_WALL, CBORE_DEPTH, BACK_WALL - CBORE_DEPTH, CBORE_FLOOR_MIN))
        if SPACER_L <= 0:
            fail.append('Derived spacer length is %.2f. BACK_WALL + BACK_GAP (%.2f) must exceed '
                        'the D-cut start at z=%.2f.' % (SPACER_L, zA0, z_dsh0))
        if HEAD_POCKET_DEPTH > t - 1.0:
            fail.append('Head pocket %.2f in a %.1f mm disc leaves under 1 mm of material.'
                        % (HEAD_POCKET_DEPTH, t))

        if TOL_OUT_HOLE <= 0.0:
            warn.append('TOL_OUT_HOLE is 0 - the output hole is the exact geometric minimum '
                        'and WILL bind if the printer runs undersize. Use 0.10-0.25.')
        if HEAD_POCKET_DEPTH < SCREW_HEAD_H:
            warn.append('Pocket %.2f is SHALLOWER than the head (%.2f) - the head will stand '
                        '%.2f mm proud and hold disc A off disc B.'
                        % (HEAD_POCKET_DEPTH, SCREW_HEAD_H, SCREW_HEAD_H - HEAD_POCKET_DEPTH))
        if shank_top_ifseated < z_cav1 - 0.01:
            warn.append('CLAMPING RISK: with a %.2f pocket the head bottoms out %.2f mm before '
                        'the thread shoulder reaches the flange, so tightening squeezes the discs. '
                        'You said you will leave it deliberately loose - fine, but for zero-clamp '
                        'either use a pocket of %.2f mm, or a screw with a %.2f mm smooth shank.'
                        % (HEAD_POCKET_DEPTH, z_cav1 - shank_top_ifseated,
                           pocket_for_no_clamp, shank_needed_for_pocket))
        if shank_top_ifseated < zB1 - 0.01:
            warn.append('Disc B would ride %.2f mm on THREAD at its top (shank ends at z=%.2f, '
                        'disc B ends at %.2f). That is what made it walk axially before.'
                        % (zB1 - shank_top_ifseated, shank_top_ifseated, zB1))
        if web_head < WEB_MIN:
            warn.append('HEAD POCKET WEB IS %.2f mm (want >= %.1f). The head hole is %.2f dia. '
                        'A plain %.2f mm pin would leave %.2f mm instead.'
                        % (web_head, WEB_MIN, Dh_head, SCREW_SHANK_D, web_shank))
        if engage < MIN_ENGAGE:
            warn.append('Only %.2f mm of eccentric on the D-cut (want >= %.1f).' % (engage, MIN_ENGAGE))
        if PIN_R > WALL:
            warn.append('Rollers stand %.2f mm proud of the outer wall (PIN_R %.2f > WALL %.2f) '
                        'and are being trimmed flush. Harmless as long as it is small - the '
                        'trimmed side is buried in the wall, not the running surface - but '
                        'WALL >= %.2f would give them real backing.'
                        % (PIN_R - WALL, PIN_R, WALL, PIN_R + 1.0))
        if 2 * PIN_R < 2.0:
            warn.append('Roller dia %.2f mm - consider steel dowel pins.' % (2 * PIN_R))

        if fail:
            ui.messageBox('CANNOT BUILD:\n\n- ' + '\n- '.join(fail))
            return

        # -------------------- LIVE parameters --------------------
        def get_or_add(name, val, comment):
            ex = des.userParameters.itemByName(name)
            if ex:
                return ex
            return des.userParameters.add(
                name, adsk.core.ValueInput.createByReal(val * MM), 'mm', comment)
        get_or_add('tol_disc_bore',  TOL_DISC_BORE,  'Disc centre bore clearance (cam bearing OD fit).')
        get_or_add('tol_journal',    TOL_JOURNAL,    'Eccentric journal clearance (cam bearing ID fit).')
        get_or_add('tol_big_seat',   TOL_BIG_SEAT,   'Big output bearing seat clearance.')
        get_or_add('tol_motor_bore', TOL_MOTOR_BORE, 'Motor D-shaft bore clearance.')
        get_or_add('tol_out_hole',   TOL_OUT_HOLE,   'Print allowance on the output holes, on top of pin + 2E.')

        NB = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        JO = adsk.fusion.FeatureOperations.JoinFeatureOperation
        CU = adsk.fusion.FeatureOperations.CutFeatureOperation
        P3 = adsk.core.Point3D.create
        ef = root.features.extrudeFeatures

        def ext(prof, z0, dz, op, target=None):
            inp = ef.createInput(prof, op)
            inp.startExtent = adsk.fusion.OffsetStartDefinition.create(
                adsk.core.ValueInput.createByReal(z0 * MM))
            inp.setOneSideExtent(
                adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(dz * MM)),
                adsk.fusion.ExtentDirections.PositiveExtentDirection)
            if target is not None:
                inp.participantBodies = [target]
            return ef.add(inp)

        def sk_circles(centers, rad):
            sk = root.sketches.add(root.xYConstructionPlane)
            sk.isComputeDeferred = True
            for (cx, cy) in centers:
                sk.sketchCurves.sketchCircles.addByCenterRadius(P3(cx * MM, cy * MM, 0), rad * MM)
            sk.isComputeDeferred = False
            coll = adsk.core.ObjectCollection.create()
            for p in sk.profiles:
                coll.add(p)
            return coll

        def dim_circles(centers, diam_expr, z0, dz, op, target=None):
            """One or more circles whose DIAMETER follows a parameter -> LIVE."""
            sk = root.sketches.add(root.xYConstructionPlane)
            for (cx, cy) in centers:
                c = sk.sketchCurves.sketchCircles.addByCenterRadius(P3(cx * MM, cy * MM, 0), 2.5 * MM)
                d = sk.sketchDimensions.addDiameterDimension(
                    c, P3((cx + 3.0) * MM, (cy + 3.0) * MM, 0))
                d.parameter.expression = diam_expr
            coll = adsk.core.ObjectCollection.create()
            for p in sk.profiles:
                coll.add(p)
            return ext(coll, z0, dz, op, target)

        def annulus(ro, ri, cx=0.0, cy=0.0):
            sk = root.sketches.add(root.xYConstructionPlane)
            sk.sketchCurves.sketchCircles.addByCenterRadius(P3(cx * MM, cy * MM, 0), ro * MM)
            sk.sketchCurves.sketchCircles.addByCenterRadius(P3(cx * MM, cy * MM, 0), ri * MM)
            for p in sk.profiles:
                if p.profileLoops.count == 2:
                    return p
            return None

        # -------------------- HOUSING --------------------
        if MAKE_HOUSING:
            f = ext(sk_circles([(0, 0)], R + WALL), 0.0, BACK_WALL, NB)
            hb = f.bodies.item(0); hb.name = 'Housing'
            ext(sk_circles([(0, 0)], (SPACER_D + 1.0) / 2.0), 0.0, BACK_WALL, CU, hb)
            ext(annulus(R + WALL, R), z_cav0, H_cav, JO, hb)
            rollers = [(R * math.cos(2 * math.pi * k / N), R * math.sin(2 * math.pi * k / N))
                       for k in range(N)]
            ext(sk_circles(rollers, PIN_R), z_cav0, H_cav, JO, hb)
            ext(sk_circles([(0, 0)], R + WALL), z_cav1, BIG_W, JO, hb)
            dim_circles([(0, 0)], '%g mm + tol_big_seat' % BIG_OD, z_cav1, BIG_W, CU, hb)
            hs = MOUNT_SPACING / 2.0
            mount = [(hs, hs), (-hs, hs), (-hs, -hs), (hs, -hs)]
            ext(sk_circles(mount, M3_CLEAR_D / 2.0), 0.0, BACK_WALL, CU, hb)
            ext(sk_circles(mount, CBORE_D / 2.0), BACK_WALL - CBORE_DEPTH, CBORE_DEPTH, CU, hb)
            # Trim anything that pokes past the outer wall. The roller cylinders do
            # exactly that whenever PIN_R > WALL: they sit ON the R circle, so they
            # reach R + PIN_R while the wall only reaches R + WALL.
            ext(annulus(R + WALL + 10.0, R + WALL), 0.0, z_seat1, CU, hb)

        # -------------------- DISCS --------------------
        def disc_profile_sketch(sign):
            sk = root.sketches.add(root.xYConstructionPlane)
            sk.isComputeDeferred = True
            pts = adsk.core.ObjectCollection.create()
            n = PTS_PER_LOBE * Z
            for k in range(n + 1):
                th = 2.0 * math.pi * k / n
                psi = math.atan2(math.sin((1 - N) * th), (R / (ECC * N)) - math.cos((1 - N) * th))
                x = ECC + R * math.cos(th) - Rre * math.cos(th + psi) - ECC * math.cos(N * th)
                y = -R * math.sin(th) + Rre * math.sin(th + psi) + ECC * math.sin(N * th)
                pts.add(P3(sign * x * MM, sign * y * MM, 0))
            sk.sketchCurves.sketchFittedSplines.add(pts)
            sk.isComputeDeferred = False
            return sk.profiles.item(0)

        def fillet_rim(body, r_mm, z0, z1, label):
            if r_mm <= 0:
                return
            coll = adsk.core.ObjectCollection.create()
            for face in body.faces:
                if face.geometry.surfaceType != adsk.core.SurfaceTypes.PlaneSurfaceType:
                    continue
                bb = face.boundingBox
                if abs(bb.maxPoint.z - bb.minPoint.z) > 1e-6:
                    continue
                zf = bb.minPoint.z / MM
                if min(abs(zf - z0), abs(zf - z1)) > 1e-3:
                    continue
                for loop in face.loops:
                    if loop.isOuter:
                        for e in loop.edges:
                            coll.add(e)
            if coll.count == 0:
                warn.append('%s: no outer rim edges found to fillet.' % label)
                return
            try:
                ff = root.features.filletFeatures
                fin = ff.createInput()
                fin.isRollingBallCorner = True
                fin.addConstantRadiusEdgeSet(coll, adsk.core.ValueInput.createByReal(r_mm * MM), True)
                ff.add(fin)
            except:
                warn.append('%s: %.2f mm rim fillet failed (try 0.4-0.5). Disc is still fine.'
                            % (label, r_mm))

        if MAKE_DISCS:
            def build_disc(sign, z0, name, is_A):
                f = ext(disc_profile_sketch(sign), z0, t, NB)
                b = f.bodies.item(0); b.name = name
                cx = sign * ECC
                dim_circles([(cx, 0)], '%g mm + tol_disc_bore' % CAM_OD, z0, t, CU, b)
                holes = [(cx + OUT_SCREW_R * math.cos(2 * math.pi * j / OUT_SCREW_N),
                          OUT_SCREW_R * math.sin(2 * math.pi * j / OUT_SCREW_N))
                         for j in range(OUT_SCREW_N)]
                dim_circles(holes, '%g mm + tol_out_hole' % (SCREW_SHANK_D + 2.0 * ECC),
                            z0, t, CU, b)
                if is_A and HEAD_POCKET_DEPTH > 0.01:
                    dim_circles(holes, '%g mm + tol_out_hole' % (SCREW_HEAD_D + 2.0 * ECC),
                                z0, HEAD_POCKET_DEPTH, CU, b)
                fillet_rim(b, DISC_FILLET_R, z0, z0 + t, name)
                return b
            build_disc(1, zA0, 'Cycloidal disc A', True)
            if TWO_DISCS:
                build_disc(-1, zB0, 'Cycloidal disc B', False)

        # -------------------- ECCENTRIC --------------------
        if MAKE_ECC:
            f = ext(sk_circles([(0, 0)], SPACER_D / 2.0), zsp0, SPACER_L, NB)
            ec = f.bodies.item(0); ec.name = 'Eccentric shaft'
            dim_circles([(ECC, 0)], '%g mm - tol_journal' % CAM_ID, zjA0, zjA1 - zjA0, JO, ec)
            dim_circles([(-ECC, 0)], '%g mm - tol_journal' % CAM_ID, zjB0, zjB1 - zjB0, JO, ec)
            flat_x = FLAT_TO_OPP - MOTOR_SHAFT_D / 2.0
            skb = root.sketches.add(root.xYConstructionPlane)
            c = skb.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), 2.6 * MM)
            dd = skb.sketchDimensions.addDiameterDimension(c, P3(3.0 * MM, 3.0 * MM, 0))
            dd.parameter.expression = '%g mm + tol_motor_bore' % MOTOR_SHAFT_D
            skb.sketchCurves.sketchLines.addByTwoPoints(P3(flat_x * MM, -6.0 * MM, 0),
                                                       P3(flat_x * MM, 6.0 * MM, 0))
            dprof, best = None, -1.0
            for p in skb.profiles:
                a = p.areaProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).area
                if a > best:
                    best, dprof = a, p
            ext(dprof, zsp0, ecc_len, CU, ec)

        # ---------- REFERENCE HARDWARE  (visual only - do NOT print these) ----------
        #  Bearings are drawn at NOMINAL size, no tolerances: 8 dia means 8 dia.
        #  The seats around them carry the clearance, which is the point - you can
        #  see the real fit in the 3D view instead of inferring it.
        if MAKE_HARDWARE:
            def ring(cx, cy, od, idia, z0, dz, name):
                f = ext(annulus(od / 2.0, idia / 2.0, cx, cy), z0, dz, NB)
                b = f.bodies.item(0)
                b.name = name
                return b

            def hex_prof(cx, cy, af):
                """Closed regular hexagon, ACROSS FLATS = af."""
                rc = af / math.sqrt(3.0)          # circumradius from across-flats
                sk = root.sketches.add(root.xYConstructionPlane)
                ln = sk.sketchCurves.sketchLines

                def pt(k):
                    ang = math.pi / 6.0 + k * math.pi / 3.0
                    return P3((cx + rc * math.cos(ang)) * MM, (cy + rc * math.sin(ang)) * MM, 0)

                first = ln.addByTwoPoints(pt(0), pt(1))
                prev = first
                for k in range(1, 5):
                    prev = ln.addByTwoPoints(prev.endSketchPoint, pt(k + 1))
                ln.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)
                return sk.profiles.item(0)

            # --- bearings, sitting where the discs actually put them ---
            ring(ECC, 0.0, CAM_OD, CAM_ID, zA0, CAM_W, 'REF cam bearing A')
            if TWO_DISCS:
                ring(-ECC, 0.0, CAM_OD, CAM_ID, zB0, CAM_W, 'REF cam bearing B')
            ring(0.0, 0.0, BIG_OD, BIG_ID, z_cav1, BIG_W, 'REF output bearing')

            # --- output screws, seated exactly as the report describes ---
            #  Screw circle is centred on the GEARBOX axis, not the disc centre -
            #  the discs orbit around these, which is why their holes are offset.
            h_bot = head_top_ifseated - SCREW_HEAD_H
            for j in range(OUT_SCREW_N):
                a = 2.0 * math.pi * j / OUT_SCREW_N
                sx = OUT_SCREW_R * math.cos(a)
                sy = OUT_SCREW_R * math.sin(a)
                f = ext(sk_circles([(sx, sy)], SCREW_HEAD_D / 2.0), h_bot, SCREW_HEAD_H, NB)
                sc = f.bodies.item(0)
                sc.name = 'REF screw %d' % (j + 1)
                ext(sk_circles([(sx, sy)], SCREW_SHANK_D / 2.0),
                    head_top_ifseated, SCREW_SHANK_L, JO, sc)
                if SCREW_THREAD_L > 0:
                    ext(sk_circles([(sx, sy)], SCREW_THREAD_D / 2.0),
                        shank_top_ifseated, SCREW_THREAD_L, JO, sc)
                # key socket, cut into the head's motor-side face
                ext(hex_prof(sx, sy, HEX_AF), h_bot, HEX_DEPTH, CU, sc)

        app.activeViewport.fit()

        # -------------------- report --------------------
        rep = ['BUILT  %d:1  SCREWS / HEX-HEAD   (%d rollers, %d lobes)' % (RATIO, N, Z), '',
               'AXIAL STACK   (z=0 = housing back outer face)',
               '  back wall      0.00 -> %5.2f   (M3 cbore %.1f, %.1f left under the head)'
               % (BACK_WALL, CBORE_DEPTH, BACK_WALL - CBORE_DEPTH),
               '  back gap       %5.2f -> %5.2f   FIXED 2.0' % (z_cav0, zA0),
               '  disc A         %5.2f -> %5.2f' % (zA0, zA1),
               '  disc B         %5.2f -> %5.2f' % (zB0, zB1),
               '  flange face    %5.2f          total height %.2f' % (z_cav1, z_seat1),
               '',
               'MOTOR SHAFT',
               '  D-cut spans    %5.2f -> %5.2f   (%.1f standoff eats %.1f of it)'
               % (z_dsh0, z_dsh1, MOTOR_STANDOFF, MOTOR_STANDOFF - SHAFT_BOSS_L),
               '  SPACER (derived) %.2f mm  -> sits %.2f -> %.2f, exactly on the D-cut start'
               % (SPACER_L, zsp0, zsp0 + SPACER_L),
               '  eccentric %.2f long, %.2f of it engaged on the D-cut' % (ecc_len, engage),
               '',
               'OUTPUT SCREWS',
               '  shank hole   %.2f  = %.2f + 2E %.2f + tol %.2f    web %.2f'
               % (Dh_shank, SCREW_SHANK_D, 2 * ECC, TOL_OUT_HOLE, web_shank),
               '  head pocket  %.2f dia x %.2f deep                web %.2f'
               % (Dh_head, HEAD_POCKET_DEPTH, web_head),
               '  head seated  %.2f -> %.2f, shank then reaches %.2f (flange at %.2f)'
               % (head_top_ifseated - SCREW_HEAD_H, head_top_ifseated,
                  shank_top_ifseated, z_cav1),
               '',
               'REFERENCE BODIES (do not print): 2 cam bearings %g x %g x %g, output bearing'
               % (CAM_ID, CAM_OD, CAM_W),
               '  %g x %g x %g, and %d M4 screws with a %g mm hex socket.'
               % (BIG_ID, BIG_OD, BIG_W, OUT_SCREW_N, HEX_AF),
               '',
               'LIVE: tol_disc_bore / tol_journal / tol_big_seat / tol_motor_bore / tol_out_hole',
               '      (CLEAR is baked into the spline - re-run to change it)']
        if warn:
            rep += ['', 'WARNINGS:'] + ['  - ' + w for w in warn]
        ui.messageBox('\n'.join(rep))

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

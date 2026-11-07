"""Fusion 360 script: re-export the arm's link meshes for the description.

Run it from Fusion (Scripts and Add-Ins) with the full-arm design active, then:

    python description/tools/build_description.py

It writes each body in its own component frame plus the occurrence
transforms, because a body exported from an occurrence does NOT come out in
assembly coordinates. build_description.py applies them.

Edit PARTS if the assembly's occurrence or body names change.
"""
import json
import os

import adsk.core
import adsk.fusion

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "cad-raw")

PARTS = {   # (occurrence name, body name) -> mesh name used by arm.json
    ("Body 1 v3:1", "Body 1"): "base",
    ("Body 2:1", "Body 2"): "turret",
    ("Body 3:1", "Body 3"): "upper_arm",
    ("Body 4:1", "Body 4"): "forearm",
    ("Body 5:1", "Body 5"): "wrist_pitch",
    ("Body 6:1", "Body 6"): "wrist_roll",
    ("Body 7:1", "Body 7"): "tool_mount",
    ("NEMA17 40mm v3:1", "NEMA17 body"): "motor_j1",
    ("NEMA17 40mm v3:2", "NEMA17 body"): "motor_j2",
    ("NEMA17 40mm v3:3", "NEMA17 body"): "motor_j3",
}


def run(context):
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    em = des.exportManager
    os.makedirs(OUT, exist_ok=True)
    xforms, done = {"_note": "written by tools/fusion_export.py; translation in mm"}, []
    for occ in des.rootComponent.occurrences:
        for body in occ.bRepBodies:
            name = PARTS.get((occ.name, body.name))
            if not name:
                continue
            opt = em.createSTLExportOptions(body, os.path.join(OUT, name + ".stl"))
            opt.isBinaryFormat = True
            opt.unitType = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
            opt.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
            em.execute(opt)
            a = occ.transform2.asArray()
            for k in (3, 7, 11):
                a[k] *= 10.0                        # cm -> mm
            xforms[name] = {"occurrence": occ.name, "matrix": [round(v, 6) for v in a]}
            done.append(name)
    with open(os.path.join(OUT, "transforms.json"), "w") as f:
        json.dump(xforms, f, indent=2)
    missing = sorted(set(PARTS.values()) - set(done))
    app.userInterface.messageBox("Exported %d meshes to %s%s" % (
        len(done), os.path.abspath(OUT), ("\nMissing: " + ", ".join(missing)) if missing else ""))

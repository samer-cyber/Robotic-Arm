#Author-Samer Haddadin
#Description-Inverted cycloidal joint generator (v7): drum output, 1 or 2 big bearings, NEMA D-shaft input, optional arm link.
"""Fusion 360 script with a dialog.

Install: Utilities > Add-Ins > Scripts and Add-Ins > "+" next to My Scripts >
pick this folder (CycloidalJoint). Then select it and Run. Open an empty
design first, or the active one - each run adds a new component.

The maths is in cyclo_core.py (testable without Fusion: python cyclo_core.py),
the drawing in cyclo_build.py. This file is only the dialog.
"""
import importlib
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import cyclo_core      # noqa: E402
import cyclo_build     # noqa: E402

importlib.reload(cyclo_core)      # pick up edits without restarting Fusion
importlib.reload(cyclo_build)

CMD_ID = "samer_cycloidal_joint_v7"
_handlers = []
_app = adsk.core.Application.get()
_ui = _app.userInterface

SUPPORT = ["Two big bearings (stiffer, recommended)", "One big bearing (shorter)"]


def _mm(inputs, cid):
    return inputs.itemById(cid).value * 10.0          # Fusion values are cm


def read_spec(inputs) -> cyclo_core.Spec:
    sel = lambda cid: inputs.itemById(cid).selectedItem.name
    return cyclo_core.Spec(
        rollers=inputs.itemById("rollers").value,
        two_bearings=sel("support") == SUPPORT[0],
        motor=sel("motor"),
        custom_shaft_d=_mm(inputs, "shaft_d"),
        custom_flat=_mm(inputs, "flat"),
        big_bearing=sel("big"),
        cam_bearing=sel("cam"),
        screw=sel("screw"),
        screw_count=inputs.itemById("screw_count").value,
        wall=_mm(inputs, "wall"),
        clearance=_mm(inputs, "clearance"),
        auto_geometry=inputs.itemById("auto").value,
        ring_radius=_mm(inputs, "ring_r"),
        roller_radius=_mm(inputs, "roller_r"),
        eccentricity=_mm(inputs, "ecc"),
        add_link=inputs.itemById("add_link").value,
        link_length=_mm(inputs, "link_len"),
        link_skin=_mm(inputs, "link_skin"),
        link_hollow=inputs.itemById("hollow").value,
        reference_hardware=inputs.itemById("ref").value,
    )


def _dropdown(inputs, cid, label, items, default):
    dd = inputs.addDropDownCommandInput(cid, label, adsk.core.DropDownStyles.TextListDropDownStyle)
    for it in items:
        dd.listItems.add(it, it == default)
    return dd


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        try:
            cmd = args.command
            cmd.okButtonText = "Generate"
            cmd.setDialogMinimumSize(420, 600)
            inp = cmd.commandInputs
            d = cyclo_core.Spec()
            V = lambda mm: adsk.core.ValueInput.createByString("%g mm" % mm)

            g = inp.addGroupCommandInput("g_gear", "Reduction").children
            g.addIntegerSpinnerCommandInput("rollers", "Ratio (= rollers)", 8, 41, 1, d.rollers)
            _dropdown(g, "support", "Support", SUPPORT, SUPPORT[0])
            _dropdown(g, "big", "Big bearing", list(cyclo_core.BIG_BEARINGS), d.big_bearing)
            _dropdown(g, "cam", "Cam bearings", list(cyclo_core.CAM_BEARINGS), d.cam_bearing)

            m = inp.addGroupCommandInput("g_motor", "Input").children
            _dropdown(m, "motor", "Motor", list(cyclo_core.MOTORS), d.motor)
            m.addValueInput("shaft_d", "Shaft diameter", "mm", V(d.custom_shaft_d)).isVisible = False
            m.addValueInput("flat", "Flat to opposite side", "mm", V(d.custom_flat)).isVisible = False

            o = inp.addGroupCommandInput("g_out", "Output pins").children
            _dropdown(o, "screw", "Screws", list(cyclo_core.SCREWS), d.screw)
            o.addIntegerSpinnerCommandInput("screw_count", "Count (even)", 4, 12, 2, d.screw_count)

            t = inp.addGroupCommandInput("g_tooth", "Tooth geometry").children
            t.addValueInput("wall", "Drum wall", "mm", V(d.wall))
            t.addValueInput("clearance", "Profile clearance", "mm", V(d.clearance))
            t.addBoolValueInput("auto", "Auto size ring, rollers, E", True, "", d.auto_geometry)
            for cid, label, val in (("ring_r", "Ring radius", d.ring_radius),
                                    ("roller_r", "Roller radius", d.roller_radius),
                                    ("ecc", "Eccentricity", d.eccentricity)):
                t.addValueInput(cid, label, "mm", V(val)).isEnabled = False

            k = inp.addGroupCommandInput("g_link", "Arm link (optional)").children
            k.addBoolValueInput("add_link", "Add a link to the next joint", True, "", d.add_link)
            k.addValueInput("link_len", "Centre-to-centre length", "mm", V(d.link_length))
            k.addValueInput("link_skin", "Skin over the drum", "mm", V(d.link_skin))
            k.addBoolValueInput("hollow", "Open the middle", True, "", d.link_hollow)

            inp.addBoolValueInput("ref", "Reference hardware (bearings, screws, motor)", True, "",
                                  d.reference_hardware)
            box = inp.addTextBoxCommandInput("summary", "", "", 16, True)
            box.isFullWidth = True
            _refresh(inp)

            for evt, h in ((cmd.inputChanged, _Changed()), (cmd.execute, _Execute()),
                           (cmd.validateInputs, _Validate()), (cmd.destroy, _Destroy())):
                evt.add(h)
                _handlers.append(h)
        except Exception:
            _ui.messageBox("Dialog failed:\n" + traceback.format_exc())


def _refresh(inputs):
    custom = inputs.itemById("motor").selectedItem.name.startswith("Custom")
    inputs.itemById("shaft_d").isVisible = custom
    inputs.itemById("flat").isVisible = custom
    manual = not inputs.itemById("auto").value
    for cid in ("ring_r", "roller_r", "ecc"):
        inputs.itemById(cid).isEnabled = manual
    try:
        p = cyclo_core.plan(read_spec(inputs))
        text = cyclo_core.summary(p)
    except Exception as e:      # half-typed values while the user edits
        text = "(%s)" % e
    inputs.itemById("summary").text = text.replace("\n", "<br>")


class _Changed(adsk.core.InputChangedEventHandler):
    def notify(self, args):
        try:
            _refresh(args.inputs)
        except Exception:
            pass


class _Validate(adsk.core.ValidateInputsEventHandler):
    def notify(self, args):
        try:
            args.areInputsValid = cyclo_core.plan(read_spec(args.inputs)).ok
        except Exception:
            args.areInputsValid = False


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            des = adsk.fusion.Design.cast(_app.activeProduct)
            if not des:
                _ui.messageBox("Open a design first.")
                return
            p = cyclo_core.plan(read_spec(args.command.commandInputs))
            if not p.ok:
                _ui.messageBox(cyclo_core.summary(p))
                return
            occ, notes = cyclo_build.build(des, p)
            _app.activeViewport.fit()
            msg = "Built: %s\n\n%s" % (occ.component.name, cyclo_core.summary(p))
            msg += ("\n\nPrint: Drum, both carriers, Disc A, Disc B, Eccentric."
                    "\nNever print the REF bodies."
                    "\nTune fits in Modify > Change Parameters (cj_tol_*), no re-run needed.")
            if notes:
                msg += "\n\n" + "\n".join(notes)
            _ui.messageBox(msg)
        except Exception:
            _ui.messageBox("Generation failed:\n" + traceback.format_exc())


class _Destroy(adsk.core.CommandEventHandler):
    def notify(self, args):
        adsk.terminate()


def run(context):
    try:
        cdef = _ui.commandDefinitions.itemById(CMD_ID)
        if cdef:
            cdef.deleteMe()
        cdef = _ui.commandDefinitions.addButtonDefinition(
            CMD_ID, "Cycloidal joint", "Generate an inverted cycloidal joint", "")
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        _ui.messageBox("Failed:\n" + traceback.format_exc())

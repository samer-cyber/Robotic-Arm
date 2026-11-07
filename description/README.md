# Robot description

The arm's geometry in one place, used by the 3D twin, the IK
(`host/armlink/kinematics.py`) and anything that reads URDF (ROS 2, MuJoCo,
PyBullet, online URDF viewers).

| File | |
|---|---|
| `arm.json` | **source of truth**: joint positions and axes, limits, tool point, firmware sign/offset map |
| `meshes/<link>.stl` | each link's printed bodies (+ motors) merged, in the link frame, mm |
| `robotic_arm.urdf` | generated from the two above, metres. Do not edit |
| `tools/fusion_export/` | Fusion script: export bodies + placement transforms to `cad-raw/` |
| `tools/build_description.py` | `cad-raw/` + `arm.json` -> `meshes/` + URDF |

## Conventions

- Zero pose = the CAD pose, arm straight up. Every position in `arm.json` is in
  world coordinates in that pose, and every link frame is world-aligned there.
- Kinematic chain: J1 yaw, J2 / J3 / J4 pitch, J5 roll, J6 pitch.
  J2→J3 = J3→J4 = 130 mm. The wrist axes sit on the servo splines, which are
  offset 10 mm from the servo bodies, so the wrist is not spherical. The IK is
  numeric and does not care.
- `firmware_map` turns firmware angles into model angles
  (`model = sign * firmware + offset`). The signs are unverified until someone
  jogs each joint with the twin open; see [docs/calibration.md](../docs/calibration.md).
- The hand is not in the CAD, so `tcp` is an estimate.

## Regenerate after a CAD change

1. In Fusion, with `Full arm` active, run the `tools/fusion_export` script
   (add the folder under Scripts and Add-Ins first).
2. `python description/tools/build_description.py`
3. If joints moved, update their `position` in `arm.json`. The quickest way to
   read an axis off the CAD is the centre of the cylindrical face it turns on.

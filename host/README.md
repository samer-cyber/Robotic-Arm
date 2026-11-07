# Host software

Talks to `arm-fw` over USB serial. Layers, deliberately separate:

```
armlink/link.py        transport + protocol framing   <- the only code that knows the wire
armlink/arm.py         Arm API in floats and names    <- what ROS 2 / teleop / policies wrap
armlink/kinematics.py  FK + IK from description/arm.json (numpy)
armlink/fake.py        simulated controller           <- develop with no hardware
control.py             browser control panel + 3D twin
web/twin.html          the twin page (three.js)
```

## Run it

```bash
pip install pyserial numpy
python control.py                 # auto-detect port
python control.py --port COM3
python control.py --simulate      # no hardware needed
python control.py --list          # show serial ports
```

Close any serial monitor first: one program per COM port. Space bar is E-STOP
on both pages.

**Panel** (`/`): sliders and jog buttons per joint, feed, enable / stop /
E-STOP, gripper buttons, named poses (`poses.json`), and a raw command console.

**Twin** (`/twin`): the arm drawn from the CAD, mirroring the real joint
angles. Drag the gizmo and a translucent **ghost** solves IK to follow it
(position only, or position + orientation with `R`). Nothing moves until you
press **Go**, or tick *follow on release*. A red target means it is out of reach. The twin
needs internet once, to load three.js from a CDN.

## Use it as a library

```python
from armlink import connect
from armlink.kinematics import ArmModel

arm = connect()                     # or connect(simulate=True)
model = ArmModel()                  # reads ../description/arm.json

arm.enable(True)
sol = model.ik([200, 40, 180], seed=model.to_model(arm.state().by_name()))
if sol:
    arm.goto_named(model.to_firmware(sol.q), feed=0.3)
    arm.wait_idle()

arm.grip_close()                    # wind until stalled, then hold
arm.grip_open()                     # unwind as far as it wound, then stop
print(arm.grip_state())             # {'mode': 'open', 'wind': 0.0, ...}
```

Errors come back as data, not exceptions, unless you ask:

```python
r = arm.jog("J2", 999)
if not r:
    print(r.err_code, r.err_name, r.err_detail)     # 4 out-of-range J2 target=999.00 ...
```

`arm.cmd("diag units 0", strict=True)` raises `ArmError` instead.

`python -m armlink.kinematics` runs the IK self-test (300 random poses,
seeded the way a drag or a teleop stream would be).

## The simulator

`armlink/fake.py` speaks the same protocol, including the gripper verbs, with
linear joint motion and the same error codes. It is the reference: if host
code works against the simulator and fails on hardware, the difference is
mechanical, not logical. It deliberately does **not** model lost steps.

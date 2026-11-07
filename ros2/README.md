> **Status (2026-11): legacy, kept for reference.** This is the September 3-DOF
> pipeline (keyword matching + colour blobs + closed-form 3-DOF IK). It predates
> the wrist and the gear-ratio fix, and its geometry (`l1 = 180`, `l2 = 65`) and
> angle convention (shoulder 0 = horizontal) no longer match the arm. The 6-DOF
> model now lives in [`description/`](../description) and
> [`host/armlink/kinematics.py`](../host/armlink/kinematics.py); see the
> [roadmap](../docs/roadmap.md) for what replaces this. The camera setup notes
> below (OAK-D SR, usbipd, depth alignment) are still accurate.

# 3-DOF VLA pipeline — ROS 2 Jazzy on WSL 2

Text command → OAK-D SR spatial detection → analytical 3-DOF IK → ESP32 stepper motion.
No URDF, no MoveIt, no TF tree. Three nodes and a serial port.

```
"touch the red cup"
        │
        ▼
vla_commander_node ──── /vla/detect ────▶ vision_spatial_node
        │                                   │  detect blob (HSV or YOLO)
        │                                   │  median depth in bbox
        │                                   │  de-project with depth K
        │                                   │  camera frame → arm base frame
        │       ◀────── PointStamped (mm) ──┘
        ▼
 kinematics_3dof.solve_both()      closed form, both elbow branches
        │
        ▼
 armlink.Arm.goto_named()  ──────▶  /dev/ttyUSB0 @ 115200  ──▶  ESP32
```

---

## 0. What you must set before anything works

**Geometry is measured hardware** as of 2026-09-02: `l0=107.65, l1=180.00,
l2=65.00, base_offset=0`, base free to rotate ±180°. All of it lives in one
place — `vla_ws/src/arm_vla/config/vla_params.yaml`.

Camera is **overhead, looking straight down** from 610 mm above the desk, and
the transform is configured statically: no calibration run is needed to move
the arm. The one number nobody can derive for you is
`extrinsics.overhead.yaw_deg` — how the camera is rotated about its own
vertical axis. See LAUNCH.md Stage D for the 30-second check.

**Start here: [LAUNCH.md](LAUNCH.md)** is the end-to-end terminal checklist.

---

## 1. One-time setup

### Windows (PowerShell as Administrator)

```powershell
winget install dorssel.usbipd-win
```

### WSL Ubuntu

```bash
sudo apt update
sudo apt install -y usbutils python3-opencv python3-pip rsync \
  ros-jazzy-cv-bridge ros-jazzy-depthai-ros ros-jazzy-message-filters \
  ros-jazzy-image-transport python3-colcon-common-extensions \
  ros-jazzy-rosidl-default-generators ros-jazzy-ament-cmake
pip install --break-system-packages pyserial
```

Luxonis udev rules:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Serial port access for the ESP32. Group membership is only picked up by a new
login shell — in WSL that means `wsl --shutdown` from Windows and reopening, or
`newgrp dialout` to fix just the current shell:

```bash
sudo usermod -aG dialout $USER
```

Add a udev rule too. In WSL the device node is recreated every time `usbipd`
attaches it, so a `chmod 666` you did by hand is gone next session — this
survives:

```bash
echo 'KERNEL=="ttyUSB[0-9]*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-esp32-serial.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Camera config (this is the *corrected* version of the one in your notes — see §6):

```bash
cp ros2/oak_sr_config.yaml ~/oak_sr_config.yaml
```

---

## 2. Every session: attach both USB devices

WSL sees no USB until you hand it over. You need **two** devices, not one —
your notes only cover the camera.

### 2a. The camera (PowerShell as Admin, leave running)

Your fast-attach loop, unchanged — it works and it handles the `2485 → f63b`
re-enumeration:

```powershell
while ($true) {
    $devices = usbipd list | Select-String "03e7:2485|03e7:f63b"
    foreach ($line in $devices) {
        if ($line -match "^\s*([0-9]+-[0-9]+)\s+(03e7:(?:2485|f63b))\s+.*?\s+(Not shared|Shared)") {
            $busid = $matches[1]
            if ($matches[3] -eq "Not shared") { usbipd bind --busid $busid --force }
            usbipd attach --wsl --busid $busid
        }
    }
    Start-Sleep -Milliseconds 200
}
```

Worth trying once: modern `usbipd-win` has this built in.

```powershell
usbipd attach --wsl --busid <busid> --auto-attach
```

The busid does not change when the camera re-enumerates — only the product ID
does — so `--auto-attach` should cover it and saves you a spinning terminal. If
it does not, your loop is the known-good fallback.

### 2b. The ESP32 (PowerShell as Admin, once per session)

```powershell
usbipd list
```

Find the CP2102 (`10c4:ea60`) or CH340 (`1a86:7523`), then:

```powershell
usbipd bind --busid 1-4 --force
usbipd attach --wsl --busid 1-4
```

Verify both landed, in Ubuntu:

```bash
lsusb | grep -E "03e7|10c4|1a86" && ls -l /dev/ttyUSB*
```

> **Close `control.py` on Windows first.** One program per serial port. If the
> Windows GUI is holding COM7, WSL gets `/dev/ttyUSB0` but the ESP32 is already
> spoken for and you get confusing timeouts.

---

## 3. Build

```bash
bash ros2/sync.sh   # from the repo root, e.g. /mnt/d/.../Robotic Arm
```

That copies `ros2/vla_ws/src` into `~/vla_ws`, vendors `host/armlink` into the
`armlink_py` package, and builds in the right order (interfaces first — the
Python packages import the generated `srv` stubs, so they cannot build until
those exist).

Source of truth stays on `D:`. Builds happen on ext4 because `colcon` on
`/mnt/d` is slow, loses executable bits, and breaks symlink installs.

Every new terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/vla_ws/install/setup.bash
```

---

## 4. Bring-up order

Do these in order. Each one is a checkpoint that isolates the next failure.

### Step 1 — the maths, no hardware at all

```bash
python3 ~/vla_ws/src/arm_vla/arm_vla/kinematics_3dof.py
```

Verified here: round-trip error `1.4e-13 mm`, `0` IK failures, both elbow
branches, all envelope guards firing. It also prints a workspace census. With
the real geometry and full base rotation, **45% of the bounding box is
reachable**. Note the reach is a *shell*, 115–245 mm from the shoulder: L2
(65 mm) is far shorter than L1 (180 mm), so there is a dead zone in the middle
as well as a limit at the outside.

### Step 2 — camera up

```bash
ros2 launch depthai_ros_driver camera.launch.py params_file:=$HOME/oak_sr_config.yaml
```

Wait for `Camera ready!`, then list what actually exists before assuming the
configured names are right:

```bash
ros2 topic list | grep oak
```

Then **check the depth alignment** — this is the one that silently poisons
every 3D point:

```bash
ros2 topic echo --once /oak/left/camera_info   | grep frame_id
ros2 topic echo --once /oak/stereo/camera_info | grep frame_id
```

On this camera the stereo frame is `oak_right_camera_optical_frame`, so the
config detects in `/oak/right/image_raw`. If yours differs, point
`detect_image_topic` / `detect_info_topic` at whichever camera the stereo
`frame_id` matches. `Could not determine the type for the passed topic` means
that name has no publisher. The vision node names any missing topic 5 s after
startup, and warns if the frames disagree.

### Step 3 — vision alone, watch it detect

```bash
ros2 run arm_vla vision_spatial_node --ros-args --params-file ~/vla_ws/src/arm_vla/config/vla_params.yaml
```

In another terminal:

```bash
rqt_image_view /vla/debug_image
```

Put a saturated red object in view. You should see a green box and a red dot.
Then ask for one measurement:

```bash
ros2 service call /vla/detect arm_vla_interfaces/srv/DetectTarget "{label: red}"
```

With the overhead transform configured it answers `success: true` and a
`point_base` in millimetres. Check it against a tape measure — LAUNCH.md
Stage D has the exact checks.

### Step 4 — (optional) refine the transform by calibration

The static overhead model assumes the camera is perfectly square to the table.
A couple of degrees of tilt at 610 mm is ~10 mm of error at the edge of the
workspace, and the idealised model cannot represent it. If targets skew worse
the further out they are, that is what you are seeing — switch
`extrinsics.mode` to `matrix` and measure the transform instead.

Leave vision running, and in a new terminal:

```bash
ros2 run arm_vla calibrate_extrinsics --ros-args -p port:=/dev/ttyUSB0 -p label:=red
```

Jog the arm with `jog.py` or `control.py` in another terminal. Touch the object
with the tip, press Enter, repeat at **4+ well-spread positions — vary height
too, not just the table plane**. Type `done`. It runs Kabsch/Umeyama and prints
a YAML block plus the residual.

Verified here: exact recovery on clean data (`7e-13 mm`), and with 3 mm of
injected noise it recovers translation to 1.9 mm and *honestly reports* a
4.6 mm residual rather than hiding it. **If the worst residual is above ~15 mm,
do not proceed** — the tip was not really touching, the link lengths are wrong,
or your samples are clustered.

Paste the block into `vla_params.yaml` under `vision_spatial_node`, set
`extrinsics.calibrated: true`, set `extrinsics.mode: matrix`, and re-run
`sync.sh`.

### Step 5 — dry run, no motion

```bash
ros2 launch arm_vla vla_bringup.launch.py camera:=false simulate:=true
```

```bash
ros2 service call /vla/execute arm_vla_interfaces/srv/MoveToTarget \
  "{command: 'touch the red cup', dry_run: true}"
```

You get the joint angles and an FK check, and nothing moves. Confirm the angles
are plausible before you energise anything.

### Step 6 — for real

```bash
ros2 launch arm_vla vla_bringup.launch.py
```

```bash
ros2 service call /vla/execute arm_vla_interfaces/srv/MoveToTarget \
  "{command: 'touch the red cup', dry_run: false}"
```

Sequence is `home → standoff → target → dwell → standoff → home`. The standoff
is 60 mm radially inboard; going straight at a target sweeps the forearm
through whatever is in between, including the object.

One-shot CLI instead of the service:

```bash
ros2 run arm_vla vla_commander_node --ros-args --params-file ~/vla_ws/src/arm_vla/config/vla_params.yaml -- --once "touch the red cup"
```

---

## 5. Deliverables

| File | |
|---|---|
| `arm_vla/kinematics_3dof.py` | closed-form IK/FK, both branches, limits + envelope. Pure Python, runs standalone |
| `arm_vla/vision_spatial_node.py` | detect → median depth → de-project → base frame. Serves `/vla/detect` |
| `arm_vla/vla_commander_node.py` | text → detect → IK → `goto_named`. Serves `/vla/execute` |
| `arm_vla/calibrate_extrinsics.py` | **added** — solves the extrinsics you would otherwise be guessing |
| `arm_vla/vocabulary.py` | colour bands and stop words, shared by vision and commander |
| `config/vla_params.yaml` | every tunable, in one file |
| `launch/vla_bringup.launch.py` | camera + vision + commander |
| `sync.sh` | D: → WSL → build, in the correct order |

Three safety layers, all of which I confirmed fire correctly on synthetic targets:

1. **Envelope** — `z_floor`, `z_ceiling`, and a `min_radius` singularity guard.
   That last one matters: near the base axis, `atan2(y,x)` is ill-conditioned,
   so 2 mm of depth noise swings J1 by tens of degrees or flips it 180°.
2. **Joint limits** — checked on both elbow branches before anything moves.
   A point blocked elbow-up is often fine elbow-down; trying one branch and
   giving up reports reachable targets as unreachable.
3. **Agreement gate** — vision needs 5 frames agreeing within 25 mm. A moving
   or depth-unstable object is refused rather than averaged into a
   confident-looking wrong answer.

Plus: extrinsics fail closed, and any unhandled exception mid-motion triggers
E-STOP.

---

## 6. Review of your camera notes

They are good — I built against them. Three fixes and one gap.

**The `oak_sr_config.yaml` in the notes does not work.** Two independent bugs:

1. *The YAML is malformed.* Indentation is inconsistent, there is a literal tab
   before `i_pipeline_type`, `left`/`right`/`stereo` are nested **under**
   `camera` instead of beside it, and `i_publish_topic: true` sits outside its
   block entirely. It would not parse.
2. *The heredoc terminator is indented.* ` EOF` and `    EOF` with leading
   whitespace do not terminate a `<< 'EOF'` block — the terminator must be at
   column 0. Pasting that into a shell hangs waiting for input.

Since the camera demonstrably works for you, I suspect the file on disk is fine
and the notes got mangled by the Markdown round-trip. Either way, the corrected
version is at `ros2/oak_sr_config.yaml`. Diff it against yours.

**Add the depth-alignment check.** Your topic table lists
`/oak/left/camera_info` but not `/oak/stereo/camera_info`, and the de-projection
needs the *depth* intrinsics. More importantly, if depth is aligned to the right
camera while you detect in the left, pixel `(u,v)` points at different things in
the two images and every 3D point is off by the stereo baseline — a consistent,
plausible-looking, entirely wrong offset. The two `frame_id` commands in §4 step
2 settle it in five seconds. I did not hard-code an answer because I could not
verify the default for the SR; the node checks at runtime instead.

**Add the ESP32 to the session ritual.** The notes stop at the camera, but the
arm needs its own `usbipd bind` + `attach`, plus `dialout` group membership.
§2b covers it.

**Worth trying:** `usbipd attach --wsl --busid <id> --auto-attach` instead of
the polling loop. The busid survives re-enumeration, only the product ID
changes, so the built-in flag should handle the `2485 → f63b` transition.

---

## 7. Troubleshooting

| Symptom | Cause |
|---|---|
| `/vla/detect` never appears | vision node not running, or workspace not sourced |
| Detect times out, debug image is blank | topics not flowing. `ros2 topic hz /oak/right/image_raw` |
| Detect works, `only 0 good frames` | HSV bands too tight. Watch `/vla/debug_image` and loosen the saturation floor in `vocabulary.py` |
| `only N valid depth pixels` | object out of stereo range (<150 mm), textureless, or too small |
| Points wrong by a constant offset | depth aligned to the other camera — §6 |
| Points wrong and drifting with position | extrinsics under-determined. Recalibrate with more spread, especially in height |
| `too_close_to_base` | working inside the 60 mm singularity guard. Move the object out |
| `limit_base` | ±90° base limit. 26% of the box is behind it |
| `standoff ... rejected` but target solved | standoff falls inside the guard. Lower `approach_standoff_mm` |
| Arm refuses to move, no error | extrinsics still `calibrated: false` |
| Target lands on the wrong side of the arm | `overhead.yaw_deg` is 90/180/270 out |
| Every `z` off by a constant | `overhead.height_mm` is not the true optical-centre-to-desk distance |
| Serial timeouts | `control.py` still holding the port on Windows |

---

## 8. Honest limits

- **Nothing here has run against your hardware.** The maths is verified
  numerically (IK round-trip, Kabsch recovery, touch planning, envelope guards
  — all shown above). The ROS wiring is not: it has never been built by
  `colcon`, never talked to a real camera, and never opened your serial port.
  Expect to fix an import or a QoS mismatch on first run.
- **The "language" is keyword matching**, not a VLM. `"touch the red cup"`
  works because `red` is a known colour band. The place a real model slots in is
  `_resolve_label()` — swap it for something returning a grounded label or box
  and nothing downstream changes.
- **The `depthai_ros_driver` stereo alignment parameter names** in the commented
  block of `oak_sr_config.yaml` are my best understanding, not verified against
  your driver version. The runtime check is the authority.
- **Open loop.** Nothing confirms the arm arrived. `wait_idle()` reports when
  the firmware thinks it is done, which is not the same thing.
- **Eye-to-hand only.** Static overhead camera. Your notes describe eye-in-hand
  as the 6-DOF option; that needs the extrinsics to become a live TF chain
  rather than a fixed matrix, which is a different design.
- **The arm occludes the target as it approaches**, which is inherent to a
  top-down mount. Harmless here because detection commits once before motion
  starts, but visual servoing would need the camera moved off-axis.

# End-to-end launch checklist

Every command, in order, from cold machine to the arm touching something.

You will end up with **five windows**. Two PowerShell (Admin), three WSL.

| # | Window | Holds |
|---|---|---|
| P1 | PowerShell (Admin) | OAK-D auto-attach loop — stays running |
| P2 | PowerShell (Admin) | ESP32 attach, then free |
| W1 | WSL | camera driver |
| W2 | WSL | vision + commander (the launch file) |
| W3 | WSL | you type commands here |

---

## Stage 0 — flash the firmware (Windows, once per firmware change)

**Do this before any USB passthrough.** A device attached to WSL is not visible
to Windows, and PlatformIO lives on Windows. Flashing with the ESP32 handed to
WSL fails with a confusing "port not found".

PowerShell, in `firmware/`:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -t upload
```

Then, still on Windows, open the serial monitor and prove direction control
works **before** any of the ROS stack is involved:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" device monitor
```

Type these three, in order:

```
probe
enable 1
diag joint 0
```

What you must see:

- `probe` → **all three drivers online.** This is no longer optional. Direction
  is the UART shaft bit now, so a driver that will not answer cannot be told
  which way to turn, and the firmware refuses every move on that joint with
  `driver-offline` rather than guessing.
- `diag joint 0` → `shaft flipped 0 -> 1 as asked. UART direction control works.`

If the shaft bit does not flip, stop here. Nothing downstream can help.

Close the monitor. One program per port.

---

## Stage A — USB passthrough (Windows)

### P1 · OAK-D auto-attach loop (leave this window running)

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

This exists because the camera changes USB product ID (`2485` → `f63b`) while
it boots its own firmware, and WSL drops it at that moment.

### P2 · ESP32

```powershell
usbipd list
```

Find the CP2102 (`10c4:ea60`) or CH340 (`1a86:7523`) line, take its BUSID, then:

```powershell
usbipd bind --busid 1-4 --force
```

```powershell
usbipd attach --wsl --busid 1-4
```

### Verify both, in WSL

```bash
lsusb | grep -E "03e7|10c4|1a86" && ls -l /dev/ttyUSB*
```

You want a Movidius line **and** a `/dev/ttyUSB0`.

---

## Stage B — build (WSL, W3)

```bash
bash ros2/sync.sh   # from the repo root, e.g. /mnt/d/.../Robotic Arm
```

Copies the workspace to `~/vla_ws`, vendors `host/armlink`, builds interfaces
first then the Python packages. Re-run it after **any** edit on the D: drive.

Then, in every WSL window you open from here on:

```bash
source /opt/ros/jazzy/setup.bash && source ~/vla_ws/install/setup.bash
```

---

## Stage C — launch

### W1 · camera driver

```bash
ros2 launch depthai_ros_driver camera.launch.py params_file:=$HOME/oak_sr_config.yaml
```

> **Copy the config, then RESTART the camera.** Parameters are read once, at
> node startup — copying the file does nothing to a camera that is already
> running, and the topic list will look identical afterwards.
>
> ```bash
> cp ros2/oak_sr_config.yaml ~/oak_sr_config.yaml
> ```
>
> Then Ctrl-C in **W1** and relaunch.

Wait for `Camera ready!`. That message only means the device booted — it says
nothing about which nodes made it into the pipeline.

**Confirm the pipeline type actually took.** The driver logs its choice; look
for this line in the W1 output:

```
Pipeline type: Depth
```

Or ask the running node directly:

```bash
ros2 param get /oak camera.i_pipeline_type
```

If that says anything other than `Depth`, the params file was not applied —
check you passed `params_file:=$HOME/oak_sr_config.yaml` and that the node is
named `/oak` (`ros2 node list`).

**First, list what the driver actually publishes.** The topic names in
`vla_params.yaml` are the one thing never verified against your hardware:

```bash
ros2 topic list | grep oak
```

Then check the depth map and the image you detect in belong to the same camera
— if they do not, every 3D point is off by the stereo baseline. Run these
**separately**, not chained with `&&`, or a failure in the first hides the
second:

```bash
ros2 topic echo --once /oak/left/camera_info | grep frame_id
```

```bash
ros2 topic echo --once /oak/stereo/camera_info | grep frame_id
```

Same `frame_id` → good. Different → see `ros2/oak_sr_config.yaml` §alignment.

> **`Could not determine the type for the passed topic`** means nothing is
> publishing that exact name — it is a name mismatch, not a broken camera. Take
> the real name from `ros2 topic list` above and set `depth_info_topic` /
> `detect_info_topic` in `vla_params.yaml`, then re-run `sync.sh`.
>
> A missing `camera_info` is fatal rather than cosmetic: the intrinsics inside
> it are what turn a pixel plus a depth into millimetres. `vision_spatial_node`
> checks all four topics 5 s after startup and names any with no publisher.
>
> **No `/oak/stereo/*` topics at all** is a different fault: the pipeline never
> built a depth node. In order of likelihood:
>
> 1. **The camera was not restarted** after editing the config. Parameters are
>    read at startup only.
> 2. `i_pipeline_type` is not `Depth`. Check the `Pipeline type:` log line.
> 3. An unrecognised type string — the driver logs `Pipeline type [X] not
>    found in base types, trying to load as a plugin.`
>
> Note that `/oak/rgb/image_rect` is **not** a symptom. `camera.launch.py`
> starts an `image_proc::RectifyNode` whenever `rectify_rgb` is true (its
> default), and that node advertises the topic whether or not anything feeds
> it. It appears on this camera in every configuration and can be ignored.

### Which camera is the depth aligned to

Once `/oak/stereo/camera_info` exists, compare its `frame_id` against both
monochrome cameras:

```bash
for t in left right stereo; do printf "%-7s " $t; ros2 topic echo --once /oak/$t/camera_info | grep frame_id; done
```

**Answered on this hardware, 2026-09-02:**

```
left      frame_id: oak_left_camera_optical_frame
right     frame_id: oak_right_camera_optical_frame
stereo    frame_id: oak_right_camera_optical_frame
```

Depth is aligned to the **right** camera, so `vla_params.yaml` now detects in
`/oak/right/image_raw`. That matches the vendor `oak_d_sr.yaml`, which enables
only `right`. Nothing to do unless you change cameras or driver version — but
re-run this check if you do.

Get it wrong and pixel `(u,v)` means a different thing in each image, so every
3D point is out by the stereo baseline: a consistent, plausible-looking,
entirely wrong offset. The vision node compares the two `frame_id`s at startup
and refuses to be quiet about a mismatch.

### W2 · vision + commander

```bash
ros2 launch arm_vla vla_bringup.launch.py camera:=false
```

Expect, in the log:

```
geometry l0=107.65 l1=180.0 l2=65.0 offset=0.0 mm, reach 115..245 mm
arm connected on /dev/ttyUSB0. joints reported: J1, J2, J3
commander ready. service: /vla/execute
```

```
extrinsics mode=overhead camera at base (0.0, 0.0, 610.0) mm
  top-down: a target on the desk directly under the camera should read base (0, 0, ~0) mm
```

No `EXTRINSICS NOT CALIBRATED` warning — the static transform is configured.

### W3 · watch what the camera sees (optional but do it once)

```bash
rqt_image_view /vla/debug_image
```

---

## Stage D — make it move

### 1. Check the transform before trusting it

Put a saturated red object in front of the arm, roughly 200 mm out.

```bash
ros2 service call /vla/detect arm_vla_interfaces/srv/DetectTarget "{label: red}"
```

Read `point_base` and compare to a tape measure. Three checks, in order of how
badly they bite:

- **Is the yaw right?** Put the object **200 mm in front of the base, along +X**.
  It should read roughly `(200, 0, <object height>)`. If X and Y come back
  swapped or negated, the camera is bolted at a different rotation about its
  own vertical axis — try `extrinsics.overhead.yaw_deg` = `90`, `180`, `270`,
  re-run `sync.sh`, relaunch. Nothing about a height measurement tells you
  this, and getting it wrong sends targets to the wrong side of the arm with
  full confidence.

| reads as | set yaw_deg |
|---|---|
| `(200, 0)` | 0 — correct already |
| `(0, 200)` | 270 |
| `(-200, 0)` | 180 |
| `(0, -200)` | 90 |

- **Is `z` right?** An object on the desk must read `z ≈ its own height`, and a
  flat sheet of paper must read `z ≈ 0`. If everything is off by a constant,
  `extrinsics.overhead.height_mm` is wrong — it is the optical centre above the
  **desk surface**, not above the shoulder axis.
- **Is the camera centred?** If the camera is not directly above the J1 axis,
  measure the offset and set `offset_x_mm` / `offset_y_mm`. They are the camera
  centre expressed in the base frame, so a camera 120 mm toward +X reads
  `offset_x_mm: 120.0`.

### 2. Dry run — solves everything, moves nothing

```bash
ros2 service call /vla/execute arm_vla_interfaces/srv/MoveToTarget "{command: 'touch the red cup', dry_run: true}"
```

Sanity-check the angles against the pose you can see. J2 within `[-20, 135]`,
J3 within `[-150, 0]`.

### 3. For real

```bash
ros2 service call /vla/execute arm_vla_interfaces/srv/MoveToTarget "{command: 'touch the red cup', dry_run: false}"
```

Sequence: `home → standoff (60 mm inboard) → target → dwell 1 s → standoff → home`.

One-shot CLI equivalent, if you prefer not to keep the commander running:

```bash
ros2 run arm_vla vla_commander_node --ros-args --params-file ~/vla_ws/src/arm_vla/config/vla_params.yaml -- --once "touch the red cup"
```

### Stop it

There is no E-STOP service — Ctrl-C in **W2** drops the commander, which closes
the serial link and de-energises. For a real panic button keep `control.py`
open on Windows (space bar is E-STOP anywhere on that page), but remember it
needs the COM port, so it cannot run at the same time as the ROS stack.

The commander applies E-STOP itself on any unhandled exception mid-motion.

---

## Where things have to be for this to work at all

The camera looks straight down from 610 mm. The arm reaches a **shell
115–245 mm from the shoulder axis**. So:

- An object must be **within 245 mm of the base**, and no closer than about
  115 mm. There is a genuine **dead zone in the middle** — L2 (65 mm) is much
  shorter than L1 (180 mm), so the arm cannot fold tightly enough to reach
  near its own shoulder. This is not the usual "can't reach far enough".
- On the desk, an object reads **610 mm minus its own height** in depth. The
  `350–700 mm` band accepts objects up to ~260 mm tall and throws away
  everything below the desk plane, which is where the stereo holes live.
- **With `offset 0,0` the camera is directly above the base, so the arm is in
  the middle of the frame.** Put targets off to the side, and do not use a
  target colour that also appears on the arm.
- **The arm will occlude the target as it approaches** — unavoidable with a
  top-down camera. It does not matter here because `/vla/detect` takes one
  committed measurement before any motion starts, and nothing re-reads the
  camera mid-move. It does mean visual servoing would need a different mount.
- 45% of the bounding box around the arm is reachable with the base free to
  rotate fully. It was 27% with the old ±90° limit.

---

## Fast fault table

| Symptom | Cause |
|---|---|
| `Permission denied: /dev/ttyUSB0` | `newgrp dialout` in that shell, or install the udev rule in README §1. `chmod 666` works but is lost on every re-attach |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | fixed - `sync.sh` no longer runs with `set -u`. Re-run it |
| `set: pipefail: invalid option name`, message overwriting itself | the script has Windows CRLF endings. `sed -i 's/\r$//' sync.sh` then re-run |
| `Could not determine the type for the passed topic` | nothing publishes that name. `ros2 topic list \| grep oak`, then fix `*_topic` in `vla_params.yaml` |
| vision node runs but never detects | check its startup log for `NO PUBLISHER FOUND on:` |
| `colcon: command not found` | `sudo apt install -y python3-colcon-common-extensions` |
| interfaces package fails to configure | `sudo apt install -y ros-jazzy-rosidl-default-generators ros-jazzy-ament-cmake` |
| `pio run -t upload` cannot find the port | ESP32 is attached to WSL. Detach: `usbipd detach --busid 1-4` |
| `probe` shows a driver offline | direction control is dead for that joint — every move returns `driver-offline`. Check VS present, the R7 jumper, MS1/MS2 straps |
| `diag joint N` shaft does not flip | UART write not reaching the driver |
| moves return `driver-offline` but the driver was fine at boot | the bus dropped mid-session; the firmware forgets the shaft state deliberately. Run `probe` |
| move returns `timeout` on a reversal | joint would not stop within 2 s so the reversal was refused. Lower `feed` or raise accel |
| `/vla/detect` says `only N good frames` | HSV band too tight, or the object is outside the 300–1000 mm depth window |
| `outside_workspace` / `unreachable_near` | object is beyond 245 mm or inside the 115 mm dead zone |
| target lands on the wrong side of the arm | `overhead.yaw_deg` is 90/180/270 out — see the table in Stage D.1 |
| every `z` off by the same amount | `overhead.height_mm` is not the true optical-centre-to-desk distance |
| targets skew worse the further out they are | mount is not square to the table. Switch `extrinsics.mode` to `matrix` and run `calibrate_extrinsics` |
| arm goes the wrong way | shaft bit and mechanical assembly disagree. Flip `invert` for that joint: `set J1 invert 1`, then `save` |

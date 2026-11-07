# Roadmap

Small steps, each one ending in something that works and is worth a short
video. Nothing here needs a new PCB.

## Where it stands (2026-11-07)

- Six axes built: three steppers through printed cycloidal boxes, three hobby
  servos, and a tendon hand on a continuous-rotation servo.
- Firmware 1.3.0: joint-space control over USB serial, gripper verbs, and the
  correct gear ratios (16 / 21 / 16).
- A **digital twin** from the CAD, 6-DOF IK, and click-and-drag control
  (`host/control.py` then `/twin`).
- The September vision demo reached the object but landed offset. Likely
  causes, biggest first: gear ratios 5-6 % off; J2's disc B fouling its ring
  (see [gearbox notes](gearbox.md)); an approximate camera transform; the old
  180 mm geometry.

## Next: the arm does what it is told

1. **Flash 1.3.0, fix the saved ratios, check signs in the twin.**
   [calibration.md](calibration.md) steps 1-2. Takes about 20 minutes.
2. **Reprint J2's disc B** from the v7 generator (21 rollers). It is the one
   part in the arm that interferes in CAD.
3. **Calibrate the gripper** (calibration.md step 3) and measure the tool point.
4. **Repeatability test**: send the same pose 20 times and mark the fingertip.
   That spread is the accuracy floor for everything below.

## Then: teleoperation with the Quest 3 (the fun one)

A web page in the Quest browser reads hand tracking (WebXR). Your wrist pose
goes to the same IK the twin uses, and the arm follows; pinch closes the
gripper. No app, no Unity, no install. The panel already serves pages and
solves IK, so this is one HTML file and one endpoint.

- Clutch: the arm only follows while you pinch your other hand, so you can
  reposition your hand without dragging the arm along.
- The arm is slow, and that's fine: it chases the latest target and drops the rest.
- Your eyes close the loop, so teleop works before the camera is calibrated.

## Then: learning from demonstrations (the VLA path)

1. **Record** teleop sessions: camera frames + joint angles + gripper, in the
   [LeRobot](https://github.com/huggingface/lerobot) dataset format. 50-100
   demos of one simple task (pick up the block, drop it in the cup).
2. **Train ACT** on the laptop. ACT is small enough for an RTX 3050, and it
   learns the arm as it actually behaves, offsets included, as long as the arm
   is repeatable.
3. **Fine-tune SmolVLA** (450 M) on the same data, on a rented GPU for an
   evening. Inference fits the laptop. That gives you a language-conditioned
   policy: "pick up the red block".

What does **not** fit: OpenVLA (7 B) or pi0 running locally on a 4-6 GB card,
and any VLA used zero-shot on a custom arm without fine-tuning.

## Later, optional

- **LLM on top.** Claude plans ("tidy the desk"), the learned policies do the
  moving. Or, without training, Claude plus an open-vocabulary detector plus
  OAK depth plus IK. This one needs the camera calibration to be good.
- **MuJoCo** simulation from the same URDF, for testing policies without the
  arm.
- **ROS 2 / MoveIt 2** if you want collision-aware planning. The URDF is ready
  for it.

## Hardware upgrades worth their cost

| Upgrade | Why | Rough cost |
|---|---|---|
| Homing for J1-J3 (hall sensor + magnet, or a microswitch) | Today "zero" is wherever the arm sat at power-up. Every learned policy needs the same zero every day | < 5 EUR |
| AS5600 / MT6701 magnetic encoder on each stepper joint's output | Catches lost steps and gives true joint angles. The joint-node PCB already plans an MT6701 | ~3 EUR each |
| Feetech STS3215 bus servos for J4-J6 + gripper | Position and load feedback, one daisy-chained UART, natively supported by LeRobot. The gripper gets a real angle and a current limit | ~15-20 EUR each |
| Wrist camera (any small USB cam) | Imitation-learning policies work far better with a wrist view plus a scene view | ~15 EUR |
| Two-bearing gearbox on J1 and J2 | The one-bearing drums are the source of the wobble | bearings |
| 24 V motor supply | More speed headroom for the steppers (TMC2209 takes up to 28 V) | - |

On the MCU: the ESP32 is not the bottleneck. When you outgrow it, build the
CAN joint node (`hardware/electronics/can-stepper-node`), with one controller
and one encoder per joint, rather than a bigger central chip.

#include "Arm.h"
#include "TmcBus.h"
#include <math.h>
#include <stdarg.h>
#include <string.h>

Arm arm;

static StepperJoint steppers[STEPPER_COUNT];
static ServoJoint   servos[SERVO_COUNT - 1];   // J4..J6
static GripperJoint gripper;                    // GRIP, always the last joint

GripperJoint* Arm::gripperJoint() {
  return (j[JOINT_COUNT - 1] == &gripper) ? &gripper : nullptr;
}

void Arm::note(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(event, sizeof(event), fmt, ap);
  va_end(ap);
}

void Arm::begin(Print& log) {
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (!JCFG[i].present) { j[i] = nullptr; continue; }

    if (JCFG[i].kind == KIND_STEPPER && i < STEPPER_COUNT) j[i] = &steppers[i];
    else if (i == JOINT_COUNT - 1)                         j[i] = &gripper;
    else if (i >= STEPPER_COUNT)                           j[i] = &servos[i - STEPPER_COUNT];
    else                                                   j[i] = nullptr;

    if (j[i] && !j[i]->begin(i, log))
      log.printf("# !! %s did not attach - it will refuse every move\n", JCFG[i].name);
  }

  // Restore where we were when the power went off. This is open loop: nothing
  // measures the arm, so a saved position is a memory, not an observation. It
  // is right if nothing moved the arm while it was off - and these gearboxes
  // do not backdrive by hand - but it is still an assumption, so say so.
  float saved[JOINT_COUNT];
  if (positionLoad(saved, JOINT_COUNT)) {
    for (uint8_t i = 0; i < JOINT_COUNT; i++) {
      if (j[i]) { j[i]->redefinePosition(saved[i]); savedAt[i] = saved[i]; }
    }
    log.println("# positions restored from flash - UNVERIFIED. If the arm was "
                "moved while off, correct it with `teach`.");
  } else {
    log.println("# no saved positions - all joints assumed to be at 0. Use "
                "`teach` to tell the firmware where the arm actually is.");
  }
  setEnabled(false);
}

// Write positions back once motion settles, so a power cut does not lose them.
// Rate limited and change gated: NVS has finite write endurance and there is no
// reason to burn it re-storing a number that has not moved.
void Arm::autosavePositions() {
  if (isMoving()) return;
  const uint32_t now = millis();
  if (now - lastSaveMs < 10000) return;
  lastSaveMs = now;

  float cur[JOINT_COUNT];
  bool changed = false;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    cur[i] = j[i] ? j[i]->position() : savedAt[i];
    if (fabsf(cur[i] - savedAt[i]) > 0.25f) changed = true;
  }
  if (!changed) return;

  if (positionSave(cur, JOINT_COUNT)) memcpy(savedAt, cur, sizeof(savedAt));
}

void Arm::poll() {
  for (uint8_t i = 0; i < JOINT_COUNT; i++) if (j[i]) j[i]->poll();

  // A direction write that keeps failing means the UART bus died while the
  // arm was moving. FastAccelStepper will sit there inserting 2 ms pauses
  // indefinitely and the joint just stops, silently. 50 retries is 100 ms.
  for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
    if (StepperJoint::dirFaults(i) > 50) {
      note("%s direction write failing - TMC UART bus lost mid-move",
           JCFG[i].name);
      StepperJoint::clearDirFaults();
      emergencyStop();
      break;
    }
  }
  autosavePositions();
}

void Arm::setEnabled(bool on) {
  if (on && estopLatched) return;         // refuse to re-energise while latched
  energised = on;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) if (j[i]) j[i]->setEnabled(on);
}

// Latches deliberately. An e-stop that clears itself on the next command is
// not an e-stop - recovery has to be an explicit decision.
void Arm::emergencyStop() {
  stopAll(true);
  estopLatched = true;
  energised = false;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) if (j[i]) j[i]->setEnabled(false);
  note("emergency stop latched - use: estop clear");
}

void Arm::stopAll(bool immediate) {
  for (uint8_t i = 0; i < JOINT_COUNT; i++) if (j[i]) j[i]->stop(immediate);
}

uint8_t Arm::attachedCount() const {
  uint8_t n = 0;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) if (j[i] && j[i]->attached()) n++;
  return n;
}

bool Arm::isMoving() const {
  for (uint8_t i = 0; i < JOINT_COUNT; i++) if (j[i] && j[i]->isMoving()) return true;
  return false;
}

void Arm::positions(float* out) const {
  for (uint8_t i = 0; i < JOINT_COUNT; i++) out[i] = j[i] ? j[i]->position() : 0.0f;
}

ErrCode Arm::moveJoint(uint8_t i, float deg, float feed) {
  if (i >= JOINT_COUNT || !j[i]) return E_BAD_JOINT;
  if (estopLatched) return E_ESTOP;
  if (!energised)   return E_DISABLED;
  if (!j[i]->attached()) return E_NOT_ATTACHED;
  return j[i]->moveTo(deg, feed);
}

// Validate everything first, move nothing until all of it passes. A move that
// fails halfway leaves the arm in a pose nobody asked for, which is worse than
// refusing outright - and `blame` names the joint that caused the refusal so
// the host does not have to guess.
ErrCode Arm::moveAll(const float* targets, float feed, uint8_t* blame) {
  if (blame) *blame = 0xFF;
  if (estopLatched) return E_ESTOP;
  if (!energised)   return E_DISABLED;

  float longest = 0.0f;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (isnan(targets[i])) continue;
    if (!j[i] || !j[i]->attached()) { if (blame) *blame = i; return E_NOT_ATTACHED; }
    const ErrCode e = j[i]->checkTravel(targets[i]);
    if (e != E_OK) { if (blame) *blame = i; return e; }
    const float t = j[i]->estimateSeconds(targets[i], feed);
    if (t > longest) longest = t;
  }
  if (longest <= 0.0f) return E_OK;      // already there, nothing to do

  // Set every direction bit BEFORE issuing any move. Each one is a UART
  // round trip of a millisecond or two; doing them inside the loop below
  // would stagger the starts by that much per joint and break exactly the
  // coordination this function exists to provide.
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (isnan(targets[i]) || j[i]->kind() != KIND_STEPPER) continue;
    const ErrCode e = ((StepperJoint*)j[i])->prepareDirection(targets[i]);
    if (e != E_OK) { if (blame) *blame = i; return e; }
  }

  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (isnan(targets[i])) continue;
    const ErrCode e = j[i]->moveToTimed(targets[i], longest);
    if (e != E_OK) { if (blame) *blame = i; return e; }
  }
  return E_OK;
}

#include "Joint.h"
#include <math.h>

// ===========================================================================
//  GripperJoint - a continuous-rotation servo winding a tendon spool.
//
//  Everything here is timed, because nothing measures the spool:
//
//    close : drive at close_power until the wind estimate reaches 1 (that
//            takes close_ms at full power), then drop to hold_power and stay
//            there. The fingers stall on the object long before that on a
//            big part - the stall is the grip.
//    open  : drive the other way at full power until the estimate is back
//            at 0 (open_ms from fully wound), then STOP. Going further would
//            wind the tendons the other way and close the hand again.
//
//  Every drive is capped at max_run_ms whatever asked for it, so a host that
//  dies mid-command cannot leave the servo spinning.
// ===========================================================================

static const uint16_t PULSE_MIN = 500;
static const uint16_t PULSE_MAX = 2500;

bool GripperJoint::begin(uint8_t index, Print& log) {
  const bool ok = ServoJoint::begin(index, log);
  gm = G_IDLE;
  speedNow = 0.0f;
  lastMs = millis();
  return ok;
}

const char* GripperJoint::modeName() const {
  switch (gm) {
    case G_CLOSING: return "closing";
    case G_HOLDING: return "holding";
    case G_OPENING: return "opening";
    case G_RAW:     return "raw";
    default:        return wind <= 0.001f ? "open" : "stopped";
  }
}

// speed is signed: +1 = full speed in the closing direction, 0 = stand still.
void GripperJoint::drive(float speed) {
  speedNow = constrain(speed, -1.0f, 1.0f);
  const float us = (float)GCFG.neutralUs +
                   (float)GCFG.closeDir * speedNow * (float)GCFG.spanUs;
  writeMicros((uint16_t)constrain(lroundf(us), (long)PULSE_MIN, (long)PULSE_MAX));
}

// Dead reckoning of the spool. Holding does not count: the motor is stalled
// against the fingers, not turning.
void GripperJoint::integrate(uint32_t now) {
  const uint32_t dt = now - lastMs;
  lastMs = now;
  if (!energised || gm == G_HOLDING || speedNow == 0.0f) return;
  const float closeMs = GCFG.closeMs ? (float)GCFG.closeMs : 1.0f;
  const float openMs  = GCFG.openMs  ? (float)GCFG.openMs  : 1.0f;
  if (speedNow > 0.0f) wind += speedNow * (float)dt / closeMs;
  else                 wind += speedNow * (float)dt / openMs;
  wind = constrain(wind, 0.0f, 1.0f);
}

void GripperJoint::poll() {
  if (!continuous()) { ServoJoint::poll(); return; }
  const uint32_t now = millis();
  integrate(now);
  if (!energised) return;

  const bool capped = (now - startMs) >= GCFG.maxRunMs;
  switch (gm) {
    case G_CLOSING:
      if (wind >= goal - 1e-4f || capped) {
        if (goal >= 0.999f && GCFG.holdPower > 0.0f) { gm = G_HOLDING; drive(GCFG.holdPower); }
        else                                          { gm = G_IDLE;    drive(0.0f); }
      }
      break;
    case G_OPENING:
      if (wind <= goal + 1e-4f || capped) { gm = G_IDLE; drive(0.0f); }
      break;
    case G_RAW:
      if ((int32_t)(now - runUntil) >= 0) { gm = G_IDLE; drive(0.0f); }
      break;
    default:
      break;
  }
}

ErrCode GripperJoint::driveTo(float frac, float power) {
  if (channel < 0) return E_NOT_ATTACHED;
  if (!energised)  return E_DISABLED;
  const uint32_t now = millis();
  integrate(now);
  frac = constrain(frac, 0.0f, 1.0f);
  goal = frac;
  startMs = now;

  // A full close is always re-driven, so a grip that slipped is re-tightened
  // instead of being reported as "already there".
  if (frac >= 0.999f || frac > wind + 1e-3f) {
    gm = G_CLOSING;
    drive(constrain(power, 0.05f, 1.0f));
  } else if (frac < wind - 1e-3f) {
    gm = G_OPENING;
    drive(-1.0f);
  } else {
    gm = G_IDLE;
    drive(0.0f);
  }
  return E_OK;
}

ErrCode GripperJoint::close(float power) {
  if (!continuous()) return ServoJoint::moveTo(cfg().maxDeg, 1.0f);
  return driveTo(1.0f, power > 0.0f ? power : GCFG.closePower);
}

ErrCode GripperJoint::open() {
  if (!continuous()) return ServoJoint::moveTo(cfg().minDeg, 1.0f);
  return driveTo(0.0f, 1.0f);
}

ErrCode GripperJoint::run(float speed, uint32_t ms) {
  if (!continuous()) return E_UNSUPPORTED;
  if (channel < 0) return E_NOT_ATTACHED;
  if (!energised)  return E_DISABLED;
  const uint32_t now = millis();
  integrate(now);
  if (ms > GCFG.maxRunMs) ms = GCFG.maxRunMs;
  gm = G_RAW;
  startMs = now;
  runUntil = now + ms;
  drive(speed);
  return E_OK;
}

void GripperJoint::halt() {
  if (!continuous()) { ServoJoint::stop(true); return; }
  integrate(millis());
  gm = G_IDLE;
  drive(0.0f);
}

// --------------------------------------------------------------- Joint API --
void GripperJoint::setEnabled(bool on) {
  if (!continuous()) { ServoJoint::setEnabled(on); return; }
  integrate(millis());
  energised = on;
  gm = G_IDLE;
  if (on) drive(0.0f);          // neutral: powered, standing still
  else    { speedNow = 0.0f; pwmOff(); }
}

float GripperJoint::position() const {
  return continuous() ? wind * 100.0f : ServoJoint::position();
}

void GripperJoint::redefinePosition(float pct) {
  if (!continuous()) { ServoJoint::redefinePosition(pct); return; }
  wind = constrain(pct / 100.0f, 0.0f, 1.0f);
}

bool GripperJoint::isMoving() const {
  if (!continuous()) return ServoJoint::isMoving();
  return gm == G_CLOSING || gm == G_OPENING || gm == G_RAW;
}

// In continuous mode the target is "percent wound": 100 closes and holds,
// 0 opens and stops. Feed is ignored - grip force should not depend on how
// fast the arm happened to be moving.
ErrCode GripperJoint::moveTo(float pct, float feed) {
  if (!continuous()) return ServoJoint::moveTo(pct, feed);
  return driveTo(pct / 100.0f, GCFG.closePower);
}

ErrCode GripperJoint::moveToTimed(float pct, float seconds) {
  if (!continuous()) return ServoJoint::moveToTimed(pct, seconds);
  return driveTo(pct / 100.0f, GCFG.closePower);
}

float GripperJoint::estimateSeconds(float pct, float feed) const {
  if (!continuous()) return ServoJoint::estimateSeconds(pct, feed);
  const float frac = constrain(pct / 100.0f, 0.0f, 1.0f);
  if (frac > wind) return (frac - wind) * GCFG.closeMs / 1000.0f / fmaxf(GCFG.closePower, 0.05f);
  return (wind - frac) * GCFG.openMs / 1000.0f;
}

// `stop` halts motion. It must not let go of what the hand is holding - that
// is what `grip open` (or E-STOP, which de-energises everything) is for.
void GripperJoint::stop(bool immediate) {
  if (!continuous()) { ServoJoint::stop(immediate); return; }
  if (gm == G_HOLDING) return;
  halt();
}

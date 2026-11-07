#include "motion.h"
#include "drivers.h"
#include <math.h>

Motion motion;

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

bool Motion::begin(Stream &s) {
  pinMode(PIN_EN_ALL, OUTPUT);
  digitalWrite(PIN_EN_ALL, HIGH);            // ACTIVE LOW -> HIGH = disabled
  en = false;

  engine.init();
  bool allOk = true;
  for (uint8_t i = 0; i < N_AXES; i++) {
    if (pinConflict(i, PINS[i].step, false) >= 0) {
      s.printf("!! %s STEP GPIO%u is shared with another axis - not attached.\n",
               AX[i].name, PINS[i].step);
      s.println("!! Fix it with setpin, then save and reboot. Run: pinmap");
      allOk = false;
      continue;
    }
    st[i] = engine.stepperConnectToPin(PINS[i].step);
    if (!st[i]) {
      s.printf("!! %s: no free step-generator channel for GPIO%u.\n",
               AX[i].name, PINS[i].step);
      s.println("!! The S3 builds step pulses from RMT channels and there is a");
      s.println("!! hard ceiling on axis count. See docs/WIRING.md.");
      allOk = false;
      continue;
    }
    st[i]->setDirectionPin(PINS[i].dir);
    st[i]->setAutoEnable(false);             // we drive the ENable line
    st[i]->setSpeedInHz((uint32_t)maxHz(i));
    st[i]->setAcceleration((int32_t)(AX[i].maxAcc * stepsPerDeg(i)));
    st[i]->setCurrentPosition(0);
    if (PINS[i].limit >= 0) pinMode(PINS[i].limit, INPUT_PULLUP);
  }
  return allOk;
}

// ---------------------------------------------------------------- units ----
float Motion::stepsPerDeg(uint8_t i) const {
  const AxisCfg &a = AX[i];
  return (a.fullSteps * (float)a.microsteps * a.gear) / 360.0f;
}
long Motion::degToSteps(uint8_t i, float deg) const {
  return lroundf(deg * stepsPerDeg(i));
}
float Motion::stepsToDeg(uint8_t i, long steps) const {
  return (float)steps / stepsPerDeg(i);
}
float Motion::maxHz(uint8_t i) const {
  return fmaxf(1.0f, AX[i].maxVel * stepsPerDeg(i));
}

// ---------------------------------------------------------------- state ----
float Motion::jointDeg(uint8_t i) const {
  if (!connected(i)) return 0.0f;
  return stepsToDeg(i, st[i]->getCurrentPosition());
}
void Motion::jointsDeg(float q[]) const {
  for (uint8_t i = 0; i < MAX_AXES; i++) q[i] = (i < N_AXES) ? jointDeg(i) : 0.0f;
}
void Motion::setJointDeg(uint8_t i, float deg) {
  if (!connected(i)) return;
  st[i]->setCurrentPosition(degToSteps(i, deg));
}
bool Motion::axisMoving(uint8_t i) const {
  return connected(i) && st[i]->isRunning();
}
bool Motion::moving() const {
  for (uint8_t i = 0; i < N_AXES; i++) if (axisMoving(i)) return true;
  return false;
}
bool Motion::limitHit(uint8_t i) const {
  if (i >= N_AXES || PINS[i].limit < 0) return false;
  return digitalRead(PINS[i].limit) == LOW;   // switch to GND, internal pullup
}

void Motion::enable(bool on) {
  digitalWrite(PIN_EN_ALL, on ? LOW : HIGH);
  en = on;
}

// ---------------------------------------------------- limit-switch guard ---
void Motion::poll() {
  if (!en) return;
  for (uint8_t i = 0; i < N_AXES; i++) {
    if (!axisMoving(i) || PINS[i].limit < 0) continue;
    if (!limitHit(i)) continue;
    // Only a fault if we are still driving into the switch. Leaving home with
    // the switch still depressed is normal and must not trip this.
    const long delta = st[i]->targetPos() - st[i]->getCurrentPosition();
    const int8_t dir = (delta > 0) ? 1 : (delta < 0 ? -1 : 0);
    if (dir != 0 && dir == AX[i].homeDir) {
      st[i]->forceStop();
      snprintf(event, sizeof(event), "limit %s tripped, axis stopped", AX[i].name);
    }
  }
}

// ---------------------------------------------------------------- moves ----
bool Motion::moveAxisSteps(uint8_t i, long absSteps, float feed, char *err, size_t n) {
  if (!connected(i)) { snprintf(err, n, "axis %u not connected", i); return false; }
  if (!en)           { snprintf(err, n, "drivers disabled, run: en 1"); return false; }
  const float deg = stepsToDeg(i, absSteps);
  if (deg < AX[i].minDeg - 1e-3f || deg > AX[i].maxDeg + 1e-3f) {
    snprintf(err, n, "%s target %.2f deg outside [%.2f, %.2f]",
             AX[i].name, deg, AX[i].minDeg, AX[i].maxDeg);
    return false;
  }
  feed = clampf(feed, 0.001f, 1.0f);
  st[i]->setSpeedInHz((uint32_t)fmaxf(1.0f, maxHz(i) * feed));
  st[i]->setAcceleration((int32_t)fmaxf(1.0f, AX[i].maxAcc * stepsPerDeg(i) * feed));
  st[i]->applySpeedAcceleration();
  st[i]->moveTo(absSteps);
  return true;
}

// Coordinated move: every axis starts and stops together. Each axis is scaled
// down by the ratio of its own travel to the longest one, so the joint-space
// path is a straight line instead of "fast axis finishes first, then the rest".
bool Motion::moveJointsDeg(const float target[], float feed, char *err, size_t n) {
  if (!en) { snprintf(err, n, "drivers disabled, run: en 1"); return false; }

  long tgt[MAX_AXES];
  long delta[MAX_AXES];
  float tMax = 0.0f;

  for (uint8_t i = 0; i < N_AXES; i++) {
    if (!connected(i)) { snprintf(err, n, "axis %u not connected", i); return false; }
    if (target[i] < AX[i].minDeg - 1e-3f || target[i] > AX[i].maxDeg + 1e-3f) {
      snprintf(err, n, "%s target %.2f deg outside [%.2f, %.2f]",
               AX[i].name, target[i], AX[i].minDeg, AX[i].maxDeg);
      return false;
    }
    tgt[i]   = degToSteps(i, target[i]);
    delta[i] = tgt[i] - st[i]->getCurrentPosition();
    const float t = fabsf((float)delta[i]) / maxHz(i);
    if (t > tMax) tMax = t;
  }
  if (tMax <= 0.0f) { snprintf(err, n, "already there"); return true; }

  feed = clampf(feed, 0.001f, 1.0f);
  tMax /= feed;

  for (uint8_t i = 0; i < N_AXES; i++) {
    if (delta[i] == 0) continue;
    const float k  = fabsf((float)delta[i]) / (maxHz(i) * tMax);    // 0..1
    const float hz = fmaxf(1.0f, maxHz(i) * k);
    const float ac = fmaxf(1.0f, AX[i].maxAcc * stepsPerDeg(i) * k);
    st[i]->setSpeedInHz((uint32_t)hz);
    st[i]->setAcceleration((int32_t)ac);
    st[i]->applySpeedAcceleration();
  }
  for (uint8_t i = 0; i < N_AXES; i++) if (delta[i] != 0) st[i]->moveTo(tgt[i]);
  return true;
}

bool Motion::runContinuous(uint8_t i, int8_t dir, float hz, char *err, size_t n) {
  if (!connected(i)) { snprintf(err, n, "axis %u not connected", i); return false; }
  if (!en)           { snprintf(err, n, "drivers disabled, run: en 1"); return false; }
  if (hz < 1.0f) hz = 1.0f;
  st[i]->setSpeedInHz((uint32_t)hz);
  st[i]->setAcceleration((int32_t)fmaxf(1.0f, AX[i].maxAcc * stepsPerDeg(i)));
  st[i]->applySpeedAcceleration();
  if (dir >= 0) st[i]->runForward(); else st[i]->runBackward();
  return true;
}

void Motion::stopAxis(uint8_t i, bool hard) {
  if (!connected(i)) return;
  if (hard) st[i]->forceStop(); else st[i]->stopMove();
}
void Motion::stopAll(bool hard) {
  for (uint8_t i = 0; i < N_AXES; i++) stopAxis(i, hard);
}

bool Motion::waitIdle(Stream &s, uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  while (moving()) {
    poll();
    if (s.available()) {                        // any keystroke aborts
      while (s.available()) s.read();
      stopAll(false);
      s.println("# aborted");
      return false;
    }
    if (millis() - t0 > timeoutMs) {
      stopAll(true);
      s.println("# timeout, stopped");
      return false;
    }
    delay(1);
  }
  return true;
}

// --------------------------------------------------------------- homing ----
bool Motion::home(uint8_t i, bool stallguard, Stream &s) {
  if (!connected(i)) { s.printf("err axis %u not connected\n", i); return false; }
  if (!en)           { s.println("err drivers disabled, run: en 1"); return false; }
  if (!stallguard && PINS[i].limit < 0) {
    s.printf("err %s has no limit switch wired. try: home %u sg\n", AX[i].name, i);
    return false;
  }
  if (stallguard && !drivers.online[i]) {
    s.printf("err %s driver offline, StallGuard homing needs UART\n", AX[i].name);
    return false;
  }
  if (stallguard && !AX[i].stealth) {
    // StallGuard4 on the TMC2209 is a StealthChop-only feature.
    s.printf("err %s: StallGuard needs StealthChop. run: sc %u 1\n", AX[i].name, i);
    return false;
  }

  const float span = AX[i].maxDeg - AX[i].minDeg;
  const uint32_t budget =
      (uint32_t)(2000.0f * span / fmaxf(1.0f, AX[i].homeVel)) + 3000;
  const uint16_t sgTrip = (uint16_t)AX[i].sgthrs * 2;

  auto triggered = [&](uint32_t sinceStart) -> bool {
    if (!stallguard) return limitHit(i);
    if (sinceStart < 400) return false;         // ignore the accel ramp
    if (PINS[i].diag >= 0) return digitalRead(PINS[i].diag) == HIGH;
    return drivers.d(i)->SG_RESULT() <= sgTrip;
  };

  auto seek = [&](float velDeg) -> bool {
    char err[64];
    const float hz = fmaxf(1.0f, velDeg * stepsPerDeg(i));
    if (!runContinuous(i, AX[i].homeDir, hz, err, sizeof(err))) {
      s.printf("err %s\n", err);
      return false;
    }
    const uint32_t t0 = millis();
    while (millis() - t0 < budget) {
      if (triggered(millis() - t0)) { st[i]->forceStop(); return true; }
      if (s.available()) {
        while (s.available()) s.read();
        st[i]->forceStop();
        s.println("# aborted");
        return false;
      }
      delay(1);
    }
    st[i]->forceStop();
    s.printf("err %s home timed out after %lu ms\n",
             AX[i].name, (unsigned long)budget);
    return false;
  };

  s.printf("# homing %s (%s)...\n", AX[i].name,
           stallguard ? "StallGuard" : "limit switch");
  if (!seek(AX[i].homeVel)) return false;
  delay(150);

  // Back off, then creep back in. The slow second approach is what makes the
  // trip point repeatable.
  st[i]->setSpeedInHz((uint32_t)fmaxf(1.0f, AX[i].homeVel * stepsPerDeg(i)));
  st[i]->move(-AX[i].homeDir * degToSteps(i, 5.0f));
  if (!waitIdle(s, 15000)) return false;
  delay(150);

  if (!seek(AX[i].homeVel / 4.0f)) return false;
  delay(150);

  st[i]->setCurrentPosition(degToSteps(i, AX[i].homeDeg));
  s.printf("# %s homed, position set to %.2f deg\n", AX[i].name, AX[i].homeDeg);
  return true;
}

#include "servos.h"
#include <ESP32Servo.h>
#include <string.h>

Servos servos;
static Servo s_[N_SERVOS];

int servoIndex(const char *t) {
  if (!t) return -1;
  for (uint8_t i = 0; i < N_SERVOS; i++)
    if (strcasecmp(t, SV[i].name) == 0) return i;
  if (strcasecmp(t, "grip") == 0 || strcasecmp(t, "gripper") == 0) return SV_GRIP;
  if (t[0] >= '0' && t[0] <= '9') {
    const int n = atoi(t);
    if (n >= 0 && n < N_SERVOS) return n;
  }
  return -1;
}

// Joint degrees -> pulse width. The clamp is on the SERVO's own 0..range
// coordinate, which is the one that can physically damage the horn; the soft
// limits on the joint angle are checked separately in setDeg().
float Servos::toUs(uint8_t id, float deg) const {
  const ServoCfg &c = SV[id];
  float p = c.trimDeg + (c.invert ? -deg : deg);
  if (p < 0.0f)        p = 0.0f;
  if (p > c.rangeDeg)  p = c.rangeDeg;
  return c.minUs + (p / c.rangeDeg) * (float)(c.maxUs - c.minUs);
}

void Servos::begin() {
  // ESP32Servo needs timers allocated before any attach(). Four servos fit in
  // the two 16-bit timer groups with room to spare.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (uint8_t i = 0; i < N_SERVOS; i++) cmd_[i] = SV[i].parkDeg;
  attachAll();
  park();
}

void Servos::attachAll() {
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    s_[i].setPeriodHertz(50);
    s_[i].attach(SV[i].pin, SV[i].minUs, SV[i].maxUs);
  }
  attached_ = true;
}

// A detached servo stops receiving pulses and goes slack. That is the safe
// state to leave the arm in while you are handling it - a powered servo
// fighting you is how brackets get broken.
void Servos::detachAll() {
  for (uint8_t i = 0; i < N_SERVOS; i++) s_[i].detach();
  attached_ = false;
}

bool Servos::setDeg(uint8_t id, float deg, const char **why) {
  if (id >= N_SERVOS) { if (why) *why = "no such servo"; return false; }
  const ServoCfg &c = SV[id];
  if (deg < c.minDeg || deg > c.maxDeg) {
    if (why) *why = "outside soft limits";
    return false;
  }
  if (!attached_) attachAll();
  const float u = toUs(id, deg);
  us_[id]  = (uint16_t)(u + 0.5f);
  cmd_[id] = deg;
  s_[id].writeMicroseconds(us_[id]);
  return true;
}

bool Servos::grip(float pct, const char **why) {
  if (pct < 0.0f || pct > 100.0f) { if (why) *why = "grip is 0..100 percent"; return false; }
  const ServoCfg &c = SV[SV_GRIP];
  return setDeg(SV_GRIP, c.minDeg + (pct / 100.0f) * (c.maxDeg - c.minDeg), why);
}

float Servos::gripPct() const {
  const ServoCfg &c = SV[SV_GRIP];
  const float span = c.maxDeg - c.minDeg;
  if (span <= 0.0f) return 0.0f;
  return (cmd_[SV_GRIP] - c.minDeg) / span * 100.0f;
}

void Servos::park() {
  for (uint8_t i = 0; i < N_SERVOS; i++) setDeg(i, SV[i].parkDeg);
}

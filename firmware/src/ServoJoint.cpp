#include "Joint.h"
#include <math.h>

// Hobby servos want a 50 Hz frame. 16-bit resolution gives ~0.3 us of pulse
// granularity, far finer than any MG996R can resolve.
static const uint32_t SERVO_HZ  = 50;
static const uint8_t  SERVO_BITS = 16;
static const uint32_t FRAME_US  = 1000000UL / SERVO_HZ;

static int nextChannel = 0;

bool ServoJoint::begin(uint8_t index, Print& log) {
  idx = index;
  const uint8_t pin = JPIN[idx].step;      // servos use the STEP slot as PWM

  const char* why = pinUnusableReason(pin);
  if (why) {
    log.printf("# !! %s PWM GPIO%u unusable: %s\n", cfg().name, pin, why);
    return false;
  }
  if (pinOwner(idx, pin) >= 0) {
    log.printf("# !! %s PWM GPIO%u is claimed twice - not attaching\n",
               cfg().name, pin);
    return false;
  }

  channel = nextChannel++;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!ledcAttach(pin, SERVO_HZ, SERVO_BITS)) { channel = -1; return false; }
#else
  ledcSetup(channel, SERVO_HZ, SERVO_BITS);
  ledcAttachPin(pin, channel);
#endif

  // Park mid-travel but stay de-energised: a servo that snaps to a position
  // the instant power arrives can hit the arm before anyone is ready.
  target = commanded = constrain(0.0f, cfg().minDeg, cfg().maxDeg);
  energised = false;
  return true;
}

// Linear map from joint degrees to pulse width, using the two calibration
// endpoints in config. Servo travel is not guaranteed symmetric or accurate,
// so both ends are measured values rather than assumed.
uint16_t ServoJoint::pulseFor(float deg) const {
  const JointConfig& c = cfg();
  const float span = c.servoMaxDeg - c.servoMinDeg;
  const float t = (fabsf(span) < 1e-6f) ? 0.0f : (deg - c.servoMinDeg) / span;
  float us = c.servoMinUs + t * (float)(c.servoMaxUs - c.servoMinUs);
  const float lo = (float)min(c.servoMinUs, c.servoMaxUs);
  const float hi = (float)max(c.servoMinUs, c.servoMaxUs);
  us = constrain(us, lo, hi);
  return (uint16_t)lroundf(us);
}

void ServoJoint::writeMicros(uint16_t us) {
  if (channel < 0 || !energised) return;
  const uint32_t duty = (uint32_t)(((uint64_t)us << SERVO_BITS) / FRAME_US);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(JPIN[idx].step, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void ServoJoint::pwmOff() {
  if (channel < 0) return;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(JPIN[idx].step, 0);
#else
  ledcWrite(channel, 0);
#endif
}

void ServoJoint::writeAngle(float deg) {
  if (channel < 0 || !energised) return;
  writeMicros(pulseFor(cfg().invert ? -deg : deg));
  commanded = deg;
}

void ServoJoint::setEnabled(bool on) {
  energised = on;
  if (on) {
    writeAngle(target);
  } else {
    // No pulse means no holding torque - the joint goes limp, matching what
    // `en 0` does to the steppers.
    pwmOff();
  }
}

// A servo has no position feedback, so "moving" is a timed estimate, not a
// fact. It is honest about that: the estimate comes from the configured max
// velocity, and nothing here claims to know where the horn actually is.
bool ServoJoint::isMoving() const {
  return rampMs > 0 && (millis() - rampStart) < rampMs;
}

ErrCode ServoJoint::moveTo(float deg, float feed) {
  if (channel < 0) return E_NOT_ATTACHED;
  const ErrCode e = checkTravel(deg);
  if (e != E_OK) return e;

  feed = constrain(feed, 0.001f, 1.0f);
  const float seconds = estimateSeconds(deg, feed);
  rampFrom  = commanded;
  target    = deg;
  rampStart = millis();
  rampMs    = (uint32_t)(seconds * 1000.0f);
  if (rampMs == 0) writeAngle(deg);
  return E_OK;
}

ErrCode ServoJoint::moveToTimed(float deg, float seconds) {
  if (channel < 0) return E_NOT_ATTACHED;
  const ErrCode e = checkTravel(deg);
  if (e != E_OK) return e;
  rampFrom  = commanded;
  target    = deg;
  rampStart = millis();
  rampMs    = (uint32_t)(fmaxf(0.0f, seconds) * 1000.0f);
  if (rampMs == 0) writeAngle(deg);
  return E_OK;
}

// Slewing the pulse width in software is what keeps a servo from slamming to
// its target at full speed and shock-loading the wrist. It also gives the
// coordinated-move code something to synchronise against.
void ServoJoint::poll() {
  if (channel < 0 || rampMs == 0) return;
  const uint32_t dt = millis() - rampStart;
  if (dt >= rampMs) {
    writeAngle(target);
    rampMs = 0;
    return;
  }
  const float t = (float)dt / (float)rampMs;
  writeAngle(rampFrom + (target - rampFrom) * t);
}

float ServoJoint::estimateSeconds(float deg, float feed) const {
  feed = constrain(feed, 0.001f, 1.0f);
  const float dist = fabsf(deg - commanded);
  const float v = fmaxf(0.001f, cfg().maxVelDeg * feed);
  return dist / v;
}

void ServoJoint::stop(bool immediate) {
  (void)immediate;
  // Freeze wherever the ramp had got to, and stop advancing it.
  target = commanded;
  rampMs = 0;
}

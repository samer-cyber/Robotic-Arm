#pragma once
#include "config.h"
#include <FastAccelStepper.h>

class Motion {
public:
  bool begin(Stream &s);
  void poll();                                   // limit-switch watchdog

  // --- unit helpers ---
  float stepsPerDeg(uint8_t i) const;
  long  degToSteps(uint8_t i, float deg) const;
  float stepsToDeg(uint8_t i, long steps) const;
  float maxHz(uint8_t i) const;                  // steps/s at AX[i].maxVel

  // --- state ---
  bool  connected(uint8_t i) const { return i < N_AXES && st[i] != nullptr; }
  float jointDeg(uint8_t i) const;
  void  jointsDeg(float q[]) const;
  void  setJointDeg(uint8_t i, float deg);       // redefine zero, no motion
  bool  moving() const;
  bool  axisMoving(uint8_t i) const;
  bool  limitHit(uint8_t i) const;

  // --- power ---
  void  enable(bool on);
  bool  isEnabled() const { return en; }

  // --- motion ---
  bool  moveAxisSteps(uint8_t i, long absSteps, float feed, char *err, size_t n);
  bool  moveJointsDeg(const float target[], float feed, char *err, size_t n);
  bool  runContinuous(uint8_t i, int8_t dir, float hz, char *err, size_t n);
  void  stopAll(bool hard);
  void  stopAxis(uint8_t i, bool hard);
  bool  waitIdle(Stream &s, uint32_t timeoutMs = 120000);   // any key aborts

  // --- homing ---
  bool  home(uint8_t i, bool stallguard, Stream &s);

  FastAccelStepper* stepper(uint8_t i) { return (i < N_AXES) ? st[i] : nullptr; }
  const char* lastEvent() const { return event; }
  void clearEvent() { event[0] = 0; }

private:
  FastAccelStepperEngine engine = FastAccelStepperEngine();
  FastAccelStepper* st[MAX_AXES] = {nullptr};
  bool en = false;
  char event[96] = {0};
};

extern Motion motion;

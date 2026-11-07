#pragma once
#include "config.h"

// ===========================================================================
//  Hobby servos on J4 / J5 / J6 + the gripper.
//
//  These are NOT steppers and nothing about them is closed loop. There is no
//  feedback, no stall detection and no "did it get there" - you command an
//  angle and trust it. Everything here is therefore open loop by construction:
//  `pos()` returns what was last COMMANDED, not what the horn is actually
//  doing. If a servo is stalled against a hard stop the firmware cannot tell.
//
//  270 deg servos are driven with writeMicroseconds(), never write(). The
//  Arduino write() call assumes a 0..180 part and silently squashes the top
//  90 deg of travel into nothing.
// ===========================================================================
class Servos {
public:
  void  begin();
  void  attachAll();
  void  detachAll();                 // stops the pulse train - servo goes limp
  bool  attached() const { return attached_; }

  // Command a JOINT angle in degrees (0 = middle of travel). Returns false and
  // fills `why` if the soft limits reject it.
  bool  setDeg(uint8_t id, float deg, const char **why = nullptr);
  float pos(uint8_t id) const { return cmd_[id]; }
  uint16_t lastUs(uint8_t id) const { return us_[id]; }

  // Gripper convenience: 0 = closed, 100 = open, mapped onto its soft limits.
  bool  grip(float pct, const char **why = nullptr);
  float gripPct() const;

  void  park();                      // everything to its parkDeg
  float toUs(uint8_t id, float deg) const;   // exposed so the console can show it

private:
  float    cmd_[N_SERVOS] = {0};
  uint16_t us_[N_SERVOS]  = {0};
  bool     attached_      = false;
};

extern Servos servos;

// Resolve "J4" / "j5" / "grip" / "0".."3" to an index, or -1.
int servoIndex(const char *token);

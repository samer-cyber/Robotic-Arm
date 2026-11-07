#pragma once
#include "Joint.h"

// ===========================================================================
//  The arm as a whole: owns the joints, coordinates moves across them, and
//  holds the safety state.
//
//  Coordinated motion here means every joint STARTS and FINISHES together.
//  The alternative - each joint running at its own top speed - makes the
//  fastest axis finish early and the arm lurch through corners. So the slowest
//  joint sets the duration and every other joint is stretched to match it.
// ===========================================================================
class Arm {
public:
  void begin(Print& log);
  void poll();

  Joint* joint(uint8_t i) { return (i < JOINT_COUNT) ? j[i] : nullptr; }
  GripperJoint* gripperJoint();          // nullptr if GRIP is not built
  uint8_t count() const { return JOINT_COUNT; }

  // --- safety ---
  void    setEnabled(bool on);
  bool    isEnabled() const { return energised; }
  void    emergencyStop();               // latches. clear() to recover.
  bool    isEstopped() const { return estopLatched; }
  void    clearEstop() { estopLatched = false; }
  void    stopAll(bool immediate);

  // --- motion ---
  ErrCode moveJoint(uint8_t i, float deg, float feed);
  // targets[] must have JOINT_COUNT entries; NAN means "leave this one alone".
  ErrCode moveAll(const float* targets, float feed, uint8_t* blame);
  bool    isMoving() const;
  uint8_t attachedCount() const;

  // --- state ---
  void positions(float* out) const;

  const char* lastEventText() const { return event; }
  void clearEvent() { event[0] = 0; }

private:
  void note(const char* fmt, ...);
  void autosavePositions();

  Joint*  j[JOINT_COUNT] = {nullptr};
  bool    energised    = false;
  bool    estopLatched = false;
  char    event[96]    = {0};
  uint32_t lastSaveMs  = 0;
  float    savedAt[JOINT_COUNT] = {0};
};

extern Arm arm;

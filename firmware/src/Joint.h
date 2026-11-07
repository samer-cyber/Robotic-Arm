#pragma once
#include "ArmConfig.h"
#include <FastAccelStepper.h>

// ===========================================================================
//  Error codes.
//
//  Numbered, not just worded, so a host can branch on them. Every rejection
//  the firmware makes maps to exactly one of these, and the text that comes
//  with it says what was actually wrong - never just "error".
// ===========================================================================
enum ErrCode : uint8_t {
  E_OK             = 0,
  E_SYNTAX         = 1,   // could not parse the line
  E_UNKNOWN_CMD    = 2,
  E_BAD_JOINT      = 3,   // no such joint
  E_OUT_OF_RANGE   = 4,   // target outside this joint's soft limits
  E_NOT_ATTACHED   = 5,   // joint has no working hardware behind it
  E_DISABLED       = 6,   // motors not energised
  E_DRIVER_OFFLINE = 7,   // TMC2209 not answering on the UART bus
  E_BAD_PIN        = 8,
  E_BAD_VALUE      = 9,
  E_ESTOP          = 10,  // latched emergency stop, must be cleared
  E_TIMEOUT        = 11,
  E_UNSUPPORTED    = 12,  // meaningless for this joint kind
};

const char* errName(ErrCode e);

// ===========================================================================
//  One joint. A stepper and a servo answer the same questions, so the rest of
//  the firmware never needs to know which it is talking to - that is the whole
//  point of this interface. Adding the wrist servos later must not mean
//  rewriting motion, protocol or diagnostics.
//
//  Angles are ALWAYS degrees at the joint output, after gearing. The
//  conversion to steps or microseconds lives behind this line and nowhere else.
// ===========================================================================
class Joint {
public:
  virtual ~Joint() {}

  virtual bool begin(uint8_t index, Print& log) = 0;
  virtual bool attached() const = 0;
  virtual uint8_t kind() const = 0;

  virtual void  setEnabled(bool on) = 0;
  virtual float position() const = 0;
  virtual void  redefinePosition(float deg) = 0;   // no motion, changes belief
  virtual bool  isMoving() const = 0;

  // feed is 0..1 of this joint's configured max velocity.
  virtual ErrCode moveTo(float deg, float feed) = 0;
  virtual void    stop(bool immediate) = 0;
  virtual void    poll() {}

  // How long a move to `deg` would take at `feed`, in seconds. Used to make
  // several joints start and finish together.
  virtual float estimateSeconds(float deg, float feed) const = 0;

  // Same move, but stretched to take exactly `seconds`. Returns E_OK if it
  // could be scaled; a joint that cannot go that slowly says so.
  virtual ErrCode moveToTimed(float deg, float seconds) = 0;

  uint8_t index() const { return idx; }
  const JointConfig& cfg() const { return JCFG[idx]; }
  JointConfig&       cfg()       { return JCFG[idx]; }

  // Shared limit check so every path rejects identically.
  ErrCode checkTravel(float deg) const {
    const JointConfig& c = JCFG[idx];
    if (deg < c.minDeg - 1e-3f || deg > c.maxDeg + 1e-3f) return E_OUT_OF_RANGE;
    return E_OK;
  }

protected:
  uint8_t idx = 0;
};

// ---------------------------------------------------------------------------
//  Stepper joint: FastAccelStepper generates the pulses in hardware, the
//  TMC2209 is configured over UART.
// ---------------------------------------------------------------------------
class StepperJoint : public Joint {
public:
  bool begin(uint8_t index, Print& log) override;
  bool attached() const override { return st != nullptr; }
  uint8_t kind() const override { return KIND_STEPPER; }

  void  setEnabled(bool on) override;
  float position() const override;
  void  redefinePosition(float deg) override;
  bool  isMoving() const override;

  ErrCode moveTo(float deg, float feed) override;
  void    stop(bool immediate) override;
  float   estimateSeconds(float deg, float feed) const override;
  ErrCode moveToTimed(float deg, float seconds) override;

  // Steps per degree at the joint output. The single conversion the whole
  // firmware depends on; if this is wrong, everything downstream is wrong by
  // exactly the same factor.
  float stepsPerDegree() const;
  float maxStepRate() const;      // steps/s at the configured max velocity
  long  degreesToSteps(float deg) const;

  // Resolve which way this joint must turn for a move to `deg`, and make it
  // so, over UART, before any pulses go out. Motion is refused if this
  // fails - see StepperJoint.cpp for why it is not just a convenience.
  ErrCode prepareDirection(float deg);

  // Raw pin access, for diagnostics only.
  FastAccelStepper* raw() { return st; }

  static FastAccelStepperEngine& engine();

  // Consecutive failed direction writes, per stepper. FastAccelStepper
  // responds to a failed external direction change by inserting a 2 ms
  // pause and retrying forever, so somebody has to notice.
  static uint16_t dirFaults(uint8_t i);
  static void     clearDirFaults();

private:
  FastAccelStepper* st = nullptr;
};

// ---------------------------------------------------------------------------
//  Servo joint: LEDC hardware PWM. No feedback, no ramp - a hobby servo runs
//  its own internal loop and gets there when it gets there.
// ---------------------------------------------------------------------------
class ServoJoint : public Joint {
public:
  bool begin(uint8_t index, Print& log) override;
  bool attached() const override { return channel >= 0; }
  uint8_t kind() const override { return KIND_SERVO; }

  void  setEnabled(bool on) override;
  float position() const override { return target; }
  void  redefinePosition(float deg) override { target = deg; }
  bool  isMoving() const override;

  ErrCode moveTo(float deg, float feed) override;
  void    stop(bool immediate) override;
  void    poll() override;
  float   estimateSeconds(float deg, float feed) const override;
  ErrCode moveToTimed(float deg, float seconds) override;

  uint16_t pulseFor(float deg) const;

protected:
  void writeAngle(float deg);
  void writeMicros(uint16_t us);   // raw pulse; no-op while de-energised
  void pwmOff();                   // no pulses at all: the servo goes limp

  int      channel  = -1;
  float    target   = 0.0f;   // where we have told it to go
  float    commanded= 0.0f;   // where the PWM currently says
  float    rampFrom = 0.0f;
  uint32_t rampStart= 0;
  uint32_t rampMs   = 0;
  bool     energised= false;
};

// ---------------------------------------------------------------------------
//  The gripper: a continuous-rotation servo on a tendon spool. In continuous
//  mode the "position" is the wind estimate in percent (0 open, 100 closed)
//  and every drive is timed, because nothing measures the spool. With
//  GCFG.continuous = false it behaves exactly like a ServoJoint, for the day
//  the hand gets a positional servo.
// ---------------------------------------------------------------------------
class GripperJoint : public ServoJoint {
public:
  enum Mode : uint8_t { G_IDLE, G_CLOSING, G_HOLDING, G_OPENING, G_RAW };

  bool    begin(uint8_t index, Print& log) override;
  void    setEnabled(bool on) override;
  float   position() const override;
  void    redefinePosition(float pct) override;
  bool    isMoving() const override;
  ErrCode moveTo(float pct, float feed) override;
  void    stop(bool immediate) override;
  void    poll() override;
  float   estimateSeconds(float pct, float feed) const override;
  ErrCode moveToTimed(float pct, float seconds) override;

  // The explicit API behind the `grip` command.
  ErrCode close(float power);
  ErrCode open();
  ErrCode run(float speed, uint32_t ms);   // raw drive for calibration, +=close
  void    halt();                          // neutral now, keep the estimate

  Mode        mode() const { return gm; }
  const char* modeName() const;
  float       windPct() const { return wind * 100.0f; }
  float       speed() const { return speedNow; }

private:
  ErrCode driveTo(float frac, float power);
  void    drive(float speed);              // signed, +1 = full speed closing
  void    integrate(uint32_t now);
  bool    continuous() const { return GCFG.continuous; }

  Mode     gm       = G_IDLE;
  float    wind     = 0.0f;   // 0 = slack/open, 1 = fully closed on nothing
  float    goal     = 0.0f;
  float    speedNow = 0.0f;
  uint32_t lastMs   = 0;
  uint32_t startMs  = 0;
  uint32_t runUntil = 0;
};

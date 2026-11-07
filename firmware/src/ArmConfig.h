#pragma once
#include <Arduino.h>

// ===========================================================================
//  Compile-time identity and sizes.
//
//  SCOPE NOTE: this firmware is joint-space only. No kinematics, no geometry,
//  no link lengths. Inverse kinematics runs on the host, as decided in the
//  project notes - IK is maths and belongs where there is a real CPU and a
//  real debugger. The firmware's whole job is: own the joints, convert
//  degrees to motion, respect limits, and never lie about what it did.
// ===========================================================================
#define FW_NAME        "arm-fw"
#define FW_VERSION     "1.3.0"
#define PROTO_VERSION  1

#define JOINT_COUNT    7      // J1..J6 + the gripper
#define STEPPER_COUNT  3      // J1..J3, NEMA 17 through cycloidal gearboxes
#define SERVO_COUNT    4      // J4, J5, J6 and GRIP - all hobby servos.
                              // J4 is an MG996R; J5, J6 and GRIP are MG995.
                              // J4/J5/J6 are 270 deg positional parts.
                              // GRIP is a CONTINUOUS-ROTATION (360 deg) servo
                              // winding the hand's tendons - see GripConfig.

// 115200, not 500000. A CH340 behind usbipd was dropping whole bursts at
// 500 kbaud: `state` came back with random subsets of joints, and a lost
// newline merged two rows so an unbuilt servo inherited present=1 and got
// commanded. No amount of host-side parsing recovers bytes that never
// arrived. A `state` reply is ~500 bytes: 10 ms at 500k, 43 ms at 115200,
// against a 5 Hz poll and an arm that takes seconds to move. Nothing here
// needs the speed.
#define HOST_BAUD      115200
#define TMC_BAUD       115200
#define TMC_RSENSE     0.11f

// A pin slot that is deliberately not wired to anything.
#define PIN_NONE       255

// Read GCONF back after every direction write to confirm the driver
// actually took it. Costs ~1.5 ms per direction change. Set to 0 only if
// the read-back proves unreliable on your bus - you lose the guarantee
// that a move goes the way you asked.
#define TMC_VERIFY_SHAFT 1

// UART bus to the TMC2209s. One wire, half duplex, addressed by MS1/MS2.
// GPIO16/17 are UART2's native pins on the classic ESP32 - no remap needed.
#define TMC_UART_TX    17
#define TMC_UART_RX    16

enum JointKind : uint8_t { KIND_STEPPER = 0, KIND_SERVO = 1 };

// ---------------------------------------------------------------------------
//  Pin assignment. Runtime editable and persisted, because the wiring is the
//  ground truth and a table in a header is only ever a guess about it.
// ---------------------------------------------------------------------------
struct JointPins {
  uint8_t step;      // stepper: STEP.  servo: PWM signal
  uint8_t dir;       // DEAD. The DIR pins on this hardware are damaged;
                     // direction is the TMC2209 GCONF.shaft bit over UART.
                     // Kept so the saved-config layout does not change.
  uint8_t enable;    // stepper only, active LOW, shared across all steppers
  uint8_t tmcAddr;   // 0..3, set by the MS1/MS2 straps on the module
};

// ---------------------------------------------------------------------------
//  Everything tunable about one joint. Persisted as a single blob.
// ---------------------------------------------------------------------------
struct JointConfig {
  char     name[6];
  uint8_t  kind;
  bool     present;            // is this joint physically built and wired?

  // --- degrees to motion ---
  float    motorStepsPerRev;   // 200 for a 1.8 deg stepper
  uint16_t microsteps;         // 1..256, written to the driver over UART
  float    gearRatio;          // reduction AFTER the motor. 15 means 15:1
  bool     invert;             // flip the sense of positive rotation

  // --- travel ---
  float    minDeg;
  float    maxDeg;

  // --- dynamics, expressed at the JOINT, not at the motor ---
  float    maxVelDeg;          // deg/s
  float    maxAccDeg;          // deg/s^2

  // --- TMC2209 ---
  uint16_t runCurrent_mA;      // RMS
  float    holdFactor;         // standstill current as a fraction of run
  bool     stealthChop;        // false = SpreadCycle: louder, more torque
  bool     interpolate;        // smooth coarse steps to 256 inside the driver

  // --- servo ---
  uint16_t servoMinUs;         // pulse width at servoMinDeg
  uint16_t servoMaxUs;         // pulse width at servoMaxDeg
  float    servoMinDeg;
  float    servoMaxDeg;
};

// ---------------------------------------------------------------------------
//  The gripper. A continuous-rotation servo has no position: the pulse width
//  sets SPEED and direction, and neutral_us means stop. The hand is a tendon
//  spool, so:
//
//    close  = wind the spool until the fingers stall on the object, then keep
//             a little torque on it (hold) so the grip does not relax.
//    open   = unwind for as long as it was wound, then STOP. Unwinding past
//             the slack point winds the tendons the other way and the hand
//             closes again - which is why open is timed and never "just
//             reverse until told otherwise".
//
//  `wind` (0..1) is the firmware's estimate of how far the spool is wound,
//  integrated from speed x time. It is dead reckoning, not a measurement:
//  a stall on a big object makes it over-count, so open deliberately stops a
//  little early (open_ms is calibrated against a FULL close on nothing).
//
//  Stored separately from the joint blob so adding it did not change the
//  JointConfig layout (which would have discarded every saved setting).
// ---------------------------------------------------------------------------
struct GripConfig {
  bool     continuous;   // false = treat GRIP as an ordinary positional servo
  uint16_t neutralUs;    // pulse at which the servo stands still. Trim this!
  uint16_t spanUs;       // pulse offset from neutral at full speed
  int8_t   closeDir;     // +1: closing is neutral + span. -1: neutral - span
  float    closePower;   // 0..1 speed used to close
  uint16_t closeMs;      // time at full power from open to fully closed (empty hand)
  float    holdPower;    // 0..1 torque kept on after closing. 0 = let go of the motor
  uint16_t openMs;       // time at full power from fully closed back to open
  uint16_t maxRunMs;     // hard cap on any single drive, whatever asked for it
};

extern JointConfig JCFG[JOINT_COUNT];
extern JointPins   JPIN[JOINT_COUNT];
extern GripConfig  GCFG;
extern bool        CFG_STALE;   // saved config was an older layout

void configLoadDefaults();
bool configSave();
bool configLoad();          // true if anything was restored

// Joint positions across a power cycle. Stored separately from the config blob
// so that changing the config layout can never take the position with it.
bool positionSave(const float* deg, uint8_t n);
bool positionLoad(float* deg, uint8_t n);   // true if a saved set was found

// ---------------------------------------------------------------------------
//  Pin safety. Two separate questions, deliberately kept separate:
//  is this pin usable at all on this chip, and is something else already
//  using it? Both have cost real debugging time on this project.
// ---------------------------------------------------------------------------
const char* pinUnusableReason(int gpio);        // nullptr = fine
int         pinOwner(int joint, int gpio);      // -1 = unclaimed
bool        pinAudit(Print& out);               // true = no problems found

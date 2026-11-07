#pragma once
#include <Arduino.h>

#define FW_NAME    "arm-testrig"
#define FW_VERSION "0.2.0"

// ===========================================================================
//  BOARD SELECT
//  Set by platformio.ini (-D ARM_BOARD=...). Defaults to the classic ESP32.
// ===========================================================================
#define BOARD_ESP32_WROOM 1      // ESP32-WROOM-32D / DevKit V1
#define BOARD_ESP32_S3    2      // ESP32-S3-DevKitC-1

#ifndef ARM_BOARD
#define ARM_BOARD BOARD_ESP32_WROOM
#endif

// ===========================================================================
//  BUILD SWITCHES
// ===========================================================================
#define MAX_AXES   6
#define N_AXES     3          // <- bump to 6 when the other 3 joints exist
#define HOST_BAUD  500000     // USB serial to the laptop
#define TMC_BAUD   115200     // TMC2209 single-wire UART
#define R_SENSE    0.11f      // BTT / FYSETC / Watterott TMC2209 = 0.11 ohm

// --- hobby servos: J4, J5, J6 and the gripper ------------------------------
#define N_SERVOS   4
#define SV_J4      0          // MG996R, 270 deg, PITCH. 139.982 mm from J3.
#define SV_J5      1          //         270 deg, ROLL - axis is ORTHOGONAL to
                              //         J4, 24.5 mm along the link from it.
#define SV_J6      2          // MG995,  270 deg, PITCH. 97.964 mm from J4,
                              //         so 73.464 mm past J5.
#define SV_GRIP    3          //         180 deg, the jaw. Not a kinematic joint.

struct AxisPins {
  uint8_t step;
  uint8_t dir;
  uint8_t en;
  int8_t  limit;    // -1 = not wired
  int8_t  diag;     // -1 = not wired  (TMC2209 DIAG / StallGuard output)
  uint8_t bus;      // 0 = Serial1, 1 = Serial2
  uint8_t addr;     // TMC2209 UART address 0..3, set by MS1/MS2 straps
};

#if ARM_BOARD == BOARD_ESP32_WROOM
// ===========================================================================
//  PIN MAP - ESP32-WROOM-32D
//  Avoided: 6..11 (SPI flash), 0/2/5/12/15 (strapping), 1/3 (UART0 console).
//  34/35/36/39 are INPUT ONLY with no internal pullups - limit switches there
//  need an external 10k pullup to 3V3.
// ===========================================================================
#define PIN_EN_ALL 13         // one ENable line to all drivers, ACTIVE LOW

#define TMC_A_TX   17         // Serial1 -> drivers with addr 0..3
#define TMC_A_RX   16
#define TMC_B_TX   19         // Serial2 -> only once you pass 4 axes
#define TMC_B_RX   18

// Limit switches when you fit them: J1=34, J2=35, J3=36  (+ external 10k
// pullup each, switch to GND). Put the numbers in the LIM column above.

#else
// ===========================================================================
//  PIN MAP - ESP32-S3-DevKitC-1
//  Avoided: 0/3/45/46 (strapping), 19/20 (native USB), 26..32 (SPI flash),
//  33..37 (octal PSRAM on -N16R8), 43/44 (UART0 console), 48 (RGB LED).
// ===========================================================================
#define PIN_EN_ALL 14

#define TMC_A_TX   17
#define TMC_A_RX   18
#define TMC_B_TX   1
#define TMC_B_RX   2

// (defaults for this board live in config.cpp)
// Limit switches when you fit them: J1=38, J2=39, J3=40 (internal pullups OK).

#endif

// LIM  = a MECHANICAL SWITCH you mount on the arm and wire straight to the
//        MCU. Nothing to do with the driver. -1 = none fitted yet.
// DIAG = the driver's StallGuard output pin. The modules in use here do not
//        break it out, so it is -1. Sensorless homing still works, the
//        firmware just polls SG_RESULT over UART instead. Slower, fine.
//
// PUT A 10k PULLUP FROM THE ENABLE LINE TO 3V3. The MCU pin floats during
// reset and reflash; the pullup guarantees the drivers stay disabled until
// firmware deliberately pulls the line low.

// ===========================================================================
//  PER-AXIS CONFIG  (runtime-editable, persists to NVS with `save`)
// ===========================================================================
struct AxisCfg {
  char     name[8];
  float    fullSteps;    // motor full steps / rev (1.8deg = 200)
  uint16_t microsteps;   // 1..256, set over UART
  float    gear;         // reduction AFTER the motor (5.0 = 5:1)
  bool     invert;       // flip direction sense
  float    minDeg;       // soft limits
  float    maxDeg;
  float    homeDeg;      // joint angle when the home switch trips
  int8_t   homeDir;      // -1 or +1, direction to seek home
  float    maxVel;       // deg/s
  float    maxAcc;       // deg/s^2
  float    homeVel;      // deg/s
  uint16_t runmA;        // RMS run current
  float    holdMult;     // hold current as fraction of run (0..1)
  bool     stealth;      // true = StealthChop, false = SpreadCycle
  bool     intpol;       // interpolate to 256 usteps
  uint8_t  sgthrs;       // StallGuard4 threshold (StealthChop only!)
  uint32_t tcoolthrs;    // CoolStep / StallGuard lower velocity threshold
  uint32_t tpwmthrs;     // StealthChop -> SpreadCycle changeover (0 = never)
  uint8_t  semin;        // CoolStep on/off (0 = off)
  uint8_t  semax;
};

// ===========================================================================
//  SERVO CONFIG  (runtime-editable, persists to NVS with `save`)
//
//  Angles you type are JOINT degrees with 0 = the middle of the sweep, so a
//  270 deg servo runs -135..+135 and the 180 deg gripper runs -90..+90. That
//  is what `trim` defaults to (range/2): it is the servo's own 0..range
//  position that your joint zero maps onto. Shift it to square the horn up
//  without re-cutting any brackets.
// ===========================================================================
struct ServoCfg {
  char     name[8];
  uint8_t  pin;
  float    rangeDeg;    // FULL mechanical sweep. 270 for the wrist, 180 gripper
  uint16_t minUs;       // pulse at servo position 0
  uint16_t maxUs;       // pulse at servo position rangeDeg
  float    minDeg;      // soft limits on the JOINT angle you command
  float    maxDeg;
  float    trimDeg;     // servo position that your joint 0 maps to
  bool     invert;      // flip which way positive goes
  float    parkDeg;     // where `svpark` sends it
};

struct Geometry {         // all mm. J4/J5/J6 are servos, see N_SERVOS above.
  float baseHeight;       // Z of the J2 axis above the work surface
  float baseOffset;       // radial offset from the J1 axis to the J2 axis
  float l1;               // J2 -> J3   (upper arm)
  float l2;               // J3 -> J4   (forearm) - MEASURED 139.982
  float l3;               // J4 -> J5   (24.5, roll axis sits along the link)
  float l4;               // J5 -> J6   (73.464 = the 97.964 J4->J6 minus l3)
  float l5;               // J6 -> tool tip  <<< MEASURE THIS, it is a guess
  float deskZ;            // height of the work surface. Nothing may go below.
  float deskMargin;       // how close to deskZ you are willing to get
};

// Declared OUT here on purpose: it used to live inside the WROOM #if branch,
// which meant the S3 target had no declaration at all and would not build.
extern AxisPins PINS[MAX_AXES];   // runtime editable, see `setpin`
extern AxisCfg  AX[MAX_AXES];
extern ServoCfg SV[N_SERVOS];
extern Geometry GEO;
extern bool     ELBOW_UP;      // default IK branch

void cfgLoadDefaults();
const char* pinWarning(int gpio);   // nullptr if the pin is safe to use
int  pinConflict(int forAxis, int gpio, bool isEnField);  // -1 = free
bool pinAudit(Print &o);            // prints problems, true = clean
bool cfgSave();
bool cfgLoad();

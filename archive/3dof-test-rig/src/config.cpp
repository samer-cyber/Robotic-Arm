#include "config.h"
#include <Preferences.h>
#include <string.h>

AxisCfg  AX[MAX_AXES];
ServoCfg SV[N_SERVOS];
AxisPins PINS[MAX_AXES];
Geometry GEO;
bool     ELBOW_UP = true;

static Preferences prefs;
// Written for information only. cfgLoad() deliberately does NOT gate on it -
// see the note there.
static const uint32_t CFG_MAGIC = 0xA3D0F002;

// Pins you should not hand to a stepper on this board, and why.
const char* pinWarning(int gpio) {
#if ARM_BOARD == BOARD_ESP32_WROOM
  if (gpio >= 6  && gpio <= 11) return "SPI flash - the board will not run";
  if (gpio == 12) return "STRAPPING (MTDI): if this line is HIGH at reset the "
                         "board will not boot. A DIR line WILL be high half the time";
  if (gpio == 0 || gpio == 2 || gpio == 5 || gpio == 15)
                                return "strapping pin, can block boot";
  if (gpio == 1 || gpio == 3)   return "UART0 - the USB console lives here";
  if (gpio == TMC_A_TX || gpio == TMC_A_RX) return "the TMC2209 UART bus";
  if (gpio >= 34 && gpio <= 39) return "INPUT ONLY, cannot drive STEP or DIR";
  if (gpio < 0 || gpio > 39)    return "not a GPIO on this board";
#else
  if (gpio >= 26 && gpio <= 32) return "SPI flash";
  if (gpio >= 33 && gpio <= 37) return "octal PSRAM on -N16R8 modules";
  if (gpio == 19 || gpio == 20) return "native USB";
  if (gpio == 0 || gpio == 3 || gpio == 45 || gpio == 46) return "strapping pin";
  if (gpio == 43 || gpio == 44) return "UART0 - the USB console";
  if (gpio == TMC_A_TX || gpio == TMC_A_RX) return "the TMC2209 UART bus";
  if (gpio < 0 || gpio > 48)    return "not a GPIO on this board";
#endif
  return nullptr;
}

void cfgLoadDefaults() {
  static const char* NAMES[MAX_AXES] = {"J1","J2","J3","J4","J5","J6"};

  // Starting point only. Real wiring wins - fix it with `setpin`, then `save`.
  static const AxisPins DEFAULT_PINS[MAX_AXES] = {
#if ARM_BOARD == BOARD_ESP32_WROOM
  //  STEP DIR  EN          LIM DIAG BUS ADDR
    { 25,  32,  PIN_EN_ALL, -1, -1,  0,  0 },
    { 26,  22,  PIN_EN_ALL, -1, -1,  0,  1 },
    { 27,  14,  PIN_EN_ALL, -1, -1,  0,  2 },
    {  4,  23,  PIN_EN_ALL, -1, -1,  0,  3 },
    { 21,  18,  PIN_EN_ALL, -1, -1,  1,  0 },
    { 19,  33,  PIN_EN_ALL, -1, -1,  1,  1 },
#else
    {  4,   8,  PIN_EN_ALL, -1, -1,  0,  0 },
    {  5,   9,  PIN_EN_ALL, -1, -1,  0,  1 },
    {  6,  10,  PIN_EN_ALL, -1, -1,  0,  2 },
    {  7,  11,  PIN_EN_ALL, -1, -1,  0,  3 },
    { 15,  12,  PIN_EN_ALL, -1, -1,  1,  0 },
    { 16,  13,  PIN_EN_ALL, -1, -1,  1,  1 },
#endif
  };
  memcpy(PINS, DEFAULT_PINS, sizeof(PINS));
  for (uint8_t i = 0; i < MAX_AXES; i++) {
    AxisCfg &a = AX[i];
    strncpy(a.name, NAMES[i], sizeof(a.name) - 1);
    a.name[sizeof(a.name) - 1] = 0;
    a.fullSteps  = 200.0f;
    a.microsteps = 1;      // full step. change live with: ms <ax> <n>
    a.gear       = 1.0f;     // <<< SET THIS. belt/gearbox ratio per joint.
    a.invert     = false;
    a.minDeg     = -180.0f;  // <<< SET THESE once you know your travel.
    a.maxDeg     =  180.0f;
    a.homeDeg    = 0.0f;
    a.homeDir    = -1;
    a.maxVel     = 60.0f;    // deg/s  - deliberately slow to start
    a.maxAcc     = 200.0f;   // deg/s^2
    a.homeVel    = 15.0f;
    a.runmA      = 600;      // <<< SET THIS to your motor's rating.
    a.holdMult   = 0.5f;
    a.stealth    = true;
    a.intpol     = true;   // keep ON - this is what makes full-step usable
    a.sgthrs     = 80;
    a.tcoolthrs  = 0xFFFFF;
    a.tpwmthrs   = 0;
    a.semin      = 0;
    a.semax      = 2;
  }
  // J1..J3 are steppers on cycloidal boxes and can turn all the way round, so
  // they keep the full +/-180. What actually stops them is the DESK, not a
  // joint limit - see GEO.deskZ and the `desk` command, which checks the real
  // arm geometry instead of pretending a fixed angle is safe.
  for (uint8_t i = 0; i < 3; i++) { AX[i].minDeg = -180.0f; AX[i].maxDeg = 180.0f; }

  GEO.baseHeight = 100.0f;  // <<< MEASURE
  GEO.baseOffset =   0.0f;  // <<< MEASURE
  GEO.l1         = 150.0f;  // <<< MEASURE  (J2 -> J3, upper arm)
  GEO.l2         = 139.982f;   // J3 -> J4   MEASURED
  GEO.l3         =  24.500f;   // J4 -> J5   MEASURED
  GEO.l4         =  73.464f;   // J5 -> J6   = 97.964 (J4->J6) - 24.5
  GEO.l5         =  60.0f;  // <<< MEASURE  (J6 -> tool tip). This is a guess.
  GEO.deskZ      =   0.0f;  // work surface. Raise it if the arm sits on a plinth.
  GEO.deskMargin =  10.0f;  // refuse to go within this of the desk
  ELBOW_UP = true;

  // ---------------------------------------------------------------- servos --
  // Pins picked to clear the active stepper axes, the TMC UART and every
  // strapping / flash / PSRAM pin on each board. See pinWarning() and
  // docs/WIRING.md. The J4..J6 entries in DEFAULT_PINS above are leftovers
  // from when those joints were going to be steppers - N_AXES is 3, so they
  // are never touched, and these servo pins deliberately reuse that space.
  static const uint8_t SVPIN[N_SERVOS] =
#if ARM_BOARD == BOARD_ESP32_WROOM
    { 4, 21, 23, 19 };
#else
    { 15, 16, 21, 47 };
#endif
  static const char* SVNAME[N_SERVOS] = {"J4","J5","J6","GRIP"};
  // MG996R / MG995 both answer to a 500..2500 us pulse. The 270 deg variants
  // stretch that same window across 270 deg instead of 180 - the pulse range
  // does not change, the gearing inside does.
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    ServoCfg &c = SV[i];
    strncpy(c.name, SVNAME[i], sizeof(c.name) - 1);
    c.name[sizeof(c.name) - 1] = 0;
    c.pin      = SVPIN[i];
    c.rangeDeg = (i == SV_GRIP) ? 180.0f : 270.0f;
    c.minUs    = 500;
    c.maxUs    = 2500;
    c.trimDeg  = c.rangeDeg * 0.5f;     // joint 0 = middle of the sweep
    c.invert   = false;
    c.minDeg   = -c.rangeDeg * 0.5f;
    c.maxDeg   =  c.rangeDeg * 0.5f;
    c.parkDeg  = 0.0f;
  }
  // Start the wrist conservative - open it up once you know the travel clears
  // the brackets. The gripper runs 0..+90 so `grip 0` is shut, `grip 100` open.
  SV[SV_J4].minDeg = -90.0f;  SV[SV_J4].maxDeg =  90.0f;
  SV[SV_J5].minDeg = -90.0f;  SV[SV_J5].maxDeg =  90.0f;
  SV[SV_J6].minDeg = -90.0f;  SV[SV_J6].maxDeg =  90.0f;
  SV[SV_GRIP].minDeg = 0.0f;  SV[SV_GRIP].maxDeg = 90.0f;
  SV[SV_GRIP].parkDeg = 0.0f;
}

bool cfgSave() {
  if (!prefs.begin("arm", false)) return false;
  prefs.putUInt("magic", CFG_MAGIC);
  prefs.putBytes("ax",  AX,   sizeof(AX));
  prefs.putBytes("sv",  SV,   sizeof(SV));
  prefs.putBytes("pin", PINS, sizeof(PINS));
  prefs.putBytes("geo", &GEO, sizeof(GEO));
  prefs.putBool("elbow", ELBOW_UP);
  prefs.end();
  return true;
}

// Each blob is loaded independently. An all-or-nothing magic number meant that
// adding one new field to the layout silently discarded every setting the user
// had already tuned - gear ratios, currents, geometry, the lot. Never again:
// a section that still matches its size is restored regardless of the rest.
bool cfgLoad() {
  if (!prefs.begin("arm", true)) return false;
  bool any = false;
  if (prefs.getBytesLength("ax") == sizeof(AX)) {
    prefs.getBytes("ax", AX, sizeof(AX));
    any = true;
  }
  if (prefs.getBytesLength("sv") == sizeof(SV)) {
    prefs.getBytes("sv", SV, sizeof(SV));
    any = true;
  }
  if (prefs.getBytesLength("geo") == sizeof(GEO)) {
    prefs.getBytes("geo", &GEO, sizeof(GEO));
    any = true;
  }
  if (prefs.getBytesLength("pin") == sizeof(PINS)) {
    prefs.getBytes("pin", PINS, sizeof(PINS));
    any = true;
  }
  ELBOW_UP = prefs.getBool("elbow", ELBOW_UP);
  prefs.end();
  return any;
}

// ---------------------------------------------------------------- audit ----
// Two axes sharing a STEP pin makes the step generator attach two channels to
// one GPIO, which crashes and reboots the board. Catch it before that happens.
int pinConflict(int forAxis, int gpio, bool isEnField) {
  for (uint8_t i = 0; i < N_AXES; i++) {
    if (!isEnField && PINS[i].en == (uint8_t)gpio) return i;
    if ((int)i == forAxis) continue;
    if (PINS[i].step == (uint8_t)gpio || PINS[i].dir == (uint8_t)gpio) return i;
  }
  return -1;
}

bool pinAudit(Print &o) {
  bool clean = true;
  for (uint8_t i = 0; i < N_AXES; i++) {
    // pinConflict() skips the axis being asked about, so it cannot see an axis
    // colliding with itself. That happens easily: `setpin 2 step 14` when this
    // axis already has DIR on 14 leaves both fields pointing at one pin.
    if (PINS[i].step == PINS[i].dir) {
      o.printf("# !! %s has STEP and DIR both on GPIO%u - it cannot work, and\n",
               AX[i].name, PINS[i].step);
      o.println("#    attaching a step generator to it may reboot the board.");
      clean = false;
    }
    const int pins[2] = {PINS[i].step, PINS[i].dir};
    const char *names[2] = {"STEP", "DIR"};
    for (int k = 0; k < 2; k++) {
      const char *w = pinWarning(pins[k]);
      if (w) {
        o.printf("# !! %s %s GPIO%d - %s\n", AX[i].name, names[k], pins[k], w);
        clean = false;
      }
      const int other = pinConflict(i, pins[k], false);
      if (other >= 0) {
        o.printf("# !! %s %s GPIO%d is ALSO used by %s. Two axes cannot share a\n",
                 AX[i].name, names[k], pins[k], AX[other].name);
        o.println("#    STEP or DIR line - this is what reboots the board.");
        clean = false;
      }
    }
  }
  return clean;
}

#include "ArmConfig.h"
#include <Preferences.h>
#include <string.h>

JointConfig JCFG[JOINT_COUNT];
JointPins   JPIN[JOINT_COUNT];
GripConfig  GCFG;
bool        CFG_STALE = false;

static Preferences prefs;
static const char* NVS_NS = "armfw";

// Bump this whenever JointConfig or JointPins changes shape - ANY change,
// including one that happens to keep the same sizeof.
//
// Size alone is not a safe check. Adding `bool present` after `uint8_t kind`
// dropped it into existing padding: both the old and new structs are 64 bytes,
// so a stale blob loaded cleanly and `present` inherited a padding byte that
// happened to be zero. Every joint silently vanished, `enable` set a flag with
// nothing behind it, and the drivers still reported online because they are on
// a different code path. An explicit version is the only honest check.
static const uint32_t CFG_LAYOUT = 2;

// ---------------------------------------------------------------------------
//  Defaults. These match the wiring as measured on the bench 2026-08-29:
//    J1 STEP 25 | J2 STEP 26 | J3 STEP 14 | EN 13
//
//  DIR is PIN_NONE on every stepper. The physical DIR pins (32/22/27) are
//  damaged, so direction is set through the TMC2209 shaft bit over the
//  UART bus instead. Nothing drives those GPIOs any more.
//  They are a starting point, not an authority. `setpin` overrides and saves.
// ---------------------------------------------------------------------------
void configLoadDefaults() {
  static const JointPins DEFAULT_PINS[JOINT_COUNT] = {
    // step      dir       en  addr
    {  25,  PIN_NONE,  13,  0 },   // J1 base yaw
    {  26,  PIN_NONE,  13,  1 },   // J2 shoulder pitch
    {  14,  PIN_NONE,  13,  2 },   // J3 elbow pitch
    {  18,   0,   0,  0 },   // J4 servo (MG996R, 270 deg) - PWM only
    {  19,   0,   0,  0 },   // J5 servo (MG995,  270 deg)
    {  23,   0,   0,  0 },   // J6 servo (MG995,  270 deg)
    {   4,   0,   0,  0 },   // GRIP     (MG995,  180 deg)
  };
  memcpy(JPIN, DEFAULT_PINS, sizeof(JPIN));

  static const char* NAMES[JOINT_COUNT] = {"J1","J2","J3","J4","J5","J6","GRIP"};
  // Gear ratios of the INVERTED cycloidal boxes: the pin carrier is bolted to
  // the motor and the roller ring is part of the next link. With the disc
  // held against rotation the ring turns once per N rollers, so the ratio is
  // the ROLLER count (lobes + 1) and the output turns the same way as the
  // motor. The "15:1" boxes have 16 rollers, the "20:1" box has 21.
  // Firmware <= 1.2.0 used 15/20/15 - every stepper joint fell ~5-6% short.
  static const float GEARS[JOINT_COUNT] = {16.0f, 21.0f, 16.0f, 1.0f, 1.0f, 1.0f, 1.0f};

  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    JointConfig& c = JCFG[i];
    memset(&c, 0, sizeof(c));
    strncpy(c.name, NAMES[i], sizeof(c.name) - 1);
    c.kind = (i < STEPPER_COUNT) ? KIND_STEPPER : KIND_SERVO;
    // Built as of 2026-10-17: the wrist servos and the gripper are fitted.
    // `present` is what hides a joint from `state`, refuses its moves and
    // frees its PWM channel - leaving it false was why the host showed
    // "servo - not built". Set it false again for anything you unbolt.
    c.present = true;

    c.motorStepsPerRev = 200.0f;
    c.microsteps       = 1;       // full step - tested better than 16 on
                                  // this arm; 16 struggled under load
    c.gearRatio        = GEARS[i];
    c.invert           = false;

    c.minDeg = -120.0f;
    c.maxDeg =  120.0f;

    c.maxVelDeg = 60.0f;
    c.maxAccDeg = 300.0f;

    // 17HS4401S is rated 1.5 A/phase. 1200 mA was found on the bench to be
    // the point where the geared joints stop losing steps, and the drivers
    // still run cool there - `driver <j>` reports otpw=0 after a soak.
    c.runCurrent_mA = 1200;
    c.holdFactor    = 0.6f;
    c.stealthChop   = false;      // SpreadCycle: a geared arm wants torque
    c.interpolate   = true;

    // 500..2500 us is the full pulse window for all four. What differs is
    // how much shaft travel that window buys: a 270 deg part sweeps -135..
    // +135, a 180 deg part -90..+90. Getting this wrong does not throw an
    // error, it just silently scales every angle you command.
    c.servoMinUs  = 500;
    c.servoMaxUs  = 2500;
    const bool deg270 = (i >= STEPPER_COUNT) && (i < JOINT_COUNT - 1);
    c.servoMinDeg = deg270 ? -135.0f : -90.0f;
    c.servoMaxDeg = deg270 ?  135.0f :  90.0f;
  }

  // The base can swing further than the pitch joints can fold.
  JCFG[0].minDeg = -180.0f; JCFG[0].maxDeg = 180.0f;

  // Wrist travel starts inside the servos' mechanical sweep on purpose. Open
  // it up with `set J4 max 130` once you have swept the joint and know where
  // it actually fouls a bracket - the servo will happily drive itself into
  // one and stall there, and nothing in the firmware can detect that.
  for (uint8_t i = STEPPER_COUNT; i < JOINT_COUNT - 1; i++) {
    JCFG[i].minDeg = -120.0f; JCFG[i].maxDeg = 120.0f;
  }
  // The hand. In continuous mode GRIP's "position" is the spool wind estimate
  // in percent: 0 = open, 100 = closed. See GripConfig in ArmConfig.h.
  JCFG[JOINT_COUNT - 1].minDeg = 0.0f;
  JCFG[JOINT_COUNT - 1].maxDeg = 100.0f;

  // Starting points, NOT measurements - calibrate them with `grip run` and
  // `grip set` (docs/calibration.md). neutral_us is the one that matters most:
  // a servo whose stop point is off by 20 us creeps forever.
  GCFG.continuous = true;
  GCFG.neutralUs  = 1500;
  GCFG.spanUs     = 500;
  GCFG.closeDir   = +1;
  GCFG.closePower = 1.0f;
  GCFG.closeMs    = 1200;
  GCFG.holdPower  = 0.30f;
  GCFG.openMs     = 1000;
  GCFG.maxRunMs   = 4000;
}

// ---------------------------------------------------------------------------
//  Persistence. Each blob is stored and restored independently and is only
//  rejected if its SIZE changed. An all-or-nothing version stamp meant that
//  adding one field silently wiped every tuned value - gear ratios, currents,
//  travel limits - and the user then spent an evening chasing a mechanical
//  fault that was actually a lost config. Never again.
// ---------------------------------------------------------------------------
bool configSave() {
  if (!prefs.begin(NVS_NS, false)) return false;
  prefs.putUInt("layout", CFG_LAYOUT);
  prefs.putBytes("jcfg", JCFG, sizeof(JCFG));
  prefs.putBytes("jpin", JPIN, sizeof(JPIN));
  prefs.putBytes("grip", &GCFG, sizeof(GCFG));
  prefs.end();
  return true;
}

bool positionSave(const float* deg, uint8_t n) {
  if (!prefs.begin(NVS_NS, false)) return false;
  prefs.putBytes("jpos", deg, sizeof(float) * n);
  prefs.end();
  return true;
}

bool positionLoad(float* deg, uint8_t n) {
  if (!prefs.begin(NVS_NS, true)) return false;
  const bool ok = prefs.getBytesLength("jpos") == sizeof(float) * n;
  if (ok) prefs.getBytes("jpos", deg, sizeof(float) * n);
  prefs.end();
  return ok;
}

bool configLoad() {
  if (!prefs.begin(NVS_NS, true)) return false;
  const uint32_t layout = prefs.getUInt("layout", 0);
  bool any = false;
  if (layout == CFG_LAYOUT) {
    if (prefs.getBytesLength("jcfg") == sizeof(JCFG)) {
      prefs.getBytes("jcfg", JCFG, sizeof(JCFG));
      any = true;
    }
    if (prefs.getBytesLength("jpin") == sizeof(JPIN)) {
      prefs.getBytes("jpin", JPIN, sizeof(JPIN));
      any = true;
    }
  }
  // Independent of the joint layout on purpose: a gripper tuning survives a
  // JointConfig change, and an old flash with no "grip" key keeps defaults.
  if (prefs.getBytesLength("grip") == sizeof(GCFG)) {
    prefs.getBytes("grip", &GCFG, sizeof(GCFG));
    any = true;
  }
  prefs.end();
  CFG_STALE = (layout != 0 && layout != CFG_LAYOUT);

  // A config saved before the UART-direction change still carries the old
  // DIR pins. Nothing reads them now, but leaving them set would make the
  // pin audit claim GPIO32/22/27 are in use and hide a real clash later.
  for (uint8_t i = 0; i < JOINT_COUNT; i++)
    if (JCFG[i].kind == KIND_STEPPER) JPIN[i].dir = PIN_NONE;

  return any;
}

// ---------------------------------------------------------------------------
//  Pin safety, for the classic ESP32-WROOM-32.
// ---------------------------------------------------------------------------
const char* pinUnusableReason(int gpio) {
  if (gpio < 0 || gpio > 39)     return "not a GPIO on this chip";
  if (gpio >= 6 && gpio <= 11)   return "SPI flash - touching it crashes the board";
  if (gpio == 12)                return "strapping (MTDI): HIGH at reset stops the board "
                                        "booting, and a DIR line is HIGH half the time";
  if (gpio == 0 || gpio == 2 || gpio == 5 || gpio == 15)
                                 return "strapping pin, can block boot";
  if (gpio == 1 || gpio == 3)    return "UART0 - the USB console";
  if (gpio == TMC_UART_TX || gpio == TMC_UART_RX)
                                 return "the TMC2209 UART bus";
  if (gpio >= 34 && gpio <= 39)  return "input only, cannot drive an output";
  return nullptr;
}

int pinOwner(int joint, int gpio) {
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if ((int)i == joint || !JCFG[i].present) continue;
    if (JPIN[i].step == (uint8_t)gpio) return i;
    if (JCFG[i].kind == KIND_STEPPER && JPIN[i].dir != PIN_NONE &&
        JPIN[i].dir == (uint8_t)gpio) return i;
  }
  return -1;
}

bool pinAudit(Print& out) {
  bool clean = true;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (!JCFG[i].present) continue;
    const bool stepper = (JCFG[i].kind == KIND_STEPPER);
    const int  pins[2] = { JPIN[i].step, JPIN[i].dir };
    const char* what[2] = { stepper ? "STEP" : "PWM", "DIR" };
    // DIR is PIN_NONE on this hardware, so there is only one pin per
    // stepper left to audit. Direction goes over the UART bus instead.
    const int n = (stepper && JPIN[i].dir != PIN_NONE) ? 2 : 1;
    for (int k = 0; k < n; k++) {
      const char* why = pinUnusableReason(pins[k]);
      if (why) {
        out.printf("# !! %s %s GPIO%d - %s\n", JCFG[i].name, what[k], pins[k], why);
        clean = false;
      }
      const int owner = pinOwner(i, pins[k]);
      if (owner >= 0) {
        out.printf("# !! %s %s GPIO%d is also claimed by %s\n",
                   JCFG[i].name, what[k], pins[k], JCFG[owner].name);
        clean = false;
      }
    }
  }
  return clean;
}

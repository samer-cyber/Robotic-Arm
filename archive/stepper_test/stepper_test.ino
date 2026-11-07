// ============================================================================
//  stepper_test.ino  -  bench test firmware for the 6-DOF arm's stepper stage
//
//  ONE firmware, TWO drivers. Pick one below. Everything above the driver layer
//  (command protocol, motion planner, step generation) is shared, so the Python
//  tools work identically against either.
//
//  Talks a plain line protocol at 115200 baud, so it works from the Arduino
//  serial monitor OR from Test/stepper_console.py. Every command answers with
//  data lines then a final "OK" or "ERR <reason>".
//
//  Type HELP over serial for the command list.
//
//  NOTE ON STEP GENERATION: this uses a micros()-timed loop, which is honest
//  for a bench rig and version-proof across Arduino-ESP32 cores. It tops out
//  around 20 kHz, which is roughly the 17HS4401S's own slew limit at 1/16
//  microstepping anyway. The REAL firmware should move to RMT/LEDC/MCPWM so
//  the CPU is free for the 2 kHz PID loop - that is a different job.
// ============================================================================

// ---------------------------------------------------------------- PICK ONE --
#define DRIVER_TMC2209
// #define DRIVER_DRV8825

// ------------------------------------------------------------- MOTOR FACTS --
#define FULL_STEPS_PER_REV  200      // 1.8 deg
#define STEP_PULSE_US       3        // DRV8825 needs >=1.9 us; TMC2209 needs 0.1

// ------------------------------------------------------------------- PINS ---
// Defaults are for an ESP32-S3 devkit on a breadboard (what you have now).
// The per-joint PCB's ESP32-C3 map is in the comment beside each line.
#define PIN_STEP    4        // PCB C3: IO4
#define PIN_DIR     5        // PCB C3: IO5
#define PIN_EN      6        // PCB C3: IO6   (active LOW; module has a 20k pull-up = off at boot)

#ifdef DRIVER_TMC2209
  #define PIN_TMC_RX  18     // PCB C3: IO10  <- module TX
  #define PIN_TMC_TX  17     // PCB C3: IO7   -> module RX (through the 1k)
#endif

#ifdef DRIVER_DRV8825
  #define PIN_MS1     15
  #define PIN_MS2     16
  #define PIN_MS3     17
  #define PIN_RST      8     // tie high to run
  #define PIN_SLP      9     // tie high to run
#endif

// ------------------------------------------------------------ TMC2209 setup -
#ifdef DRIVER_TMC2209
  #include <TMCStepper.h>
  // 0.11 ohm - read off YOUR module's schematic (R5/R6 = 0R11). If you ever use
  // a different module, check this first: it scales every current setting.
  #define R_SENSE        0.11f
  #define DRIVER_ADDRESS 0b00        // MS1 = MS2 = GND
  HardwareSerial tmcSerial(1);
  TMC2209Stepper driver(&tmcSerial, R_SENSE, DRIVER_ADDRESS);
  uint16_t currentMa = 600;
#endif

// ------------------------------------------------------------ motion state --
long     position     = 0;        // signed microsteps since ZERO
long     stepsLeft    = -1;       // -1 = continuous, 0 = idle, >0 = finite move
bool     running      = false;
int      dirSign      = +1;
float    curSps       = 0.0f;     // current rate, steps/s
float    tgtSps       = 0.0f;     // target rate, steps/s
float    accelSps2    = 4000.0f;  // steps/s^2
uint32_t lastStepUs   = 0;
uint32_t lastRampUs   = 0;
uint16_t microsteps   = 16;
bool     enabled      = false;

// ============================================================================
//  driver abstraction
// ============================================================================
void driverEnable(bool on) {
  digitalWrite(PIN_EN, on ? LOW : HIGH);   // active low on both drivers
  enabled = on;
}

bool driverSetMicrosteps(uint16_t ms) {
#ifdef DRIVER_TMC2209
  switch (ms) { case 1: case 2: case 4: case 8: case 16: case 32: case 64:
                case 128: case 256: break; default: return false; }
  driver.microsteps(ms);
#endif
#ifdef DRIVER_DRV8825
  uint8_t a, b, c;
  switch (ms) {
    case 1:  a=0; b=0; c=0; break;
    case 2:  a=1; b=0; c=0; break;
    case 4:  a=0; b=1; c=0; break;
    case 8:  a=1; b=1; c=0; break;
    case 16: a=0; b=0; c=1; break;
    case 32: a=1; b=0; c=1; break;
    default: return false;
  }
  digitalWrite(PIN_MS1, a); digitalWrite(PIN_MS2, b); digitalWrite(PIN_MS3, c);
#endif
  microsteps = ms;
  return true;
}

void driverSetup() {
  pinMode(PIN_STEP, OUTPUT); digitalWrite(PIN_STEP, LOW);
  pinMode(PIN_DIR,  OUTPUT); digitalWrite(PIN_DIR,  LOW);
  pinMode(PIN_EN,   OUTPUT); digitalWrite(PIN_EN,   HIGH);   // start DISABLED

#ifdef DRIVER_TMC2209
  tmcSerial.begin(115200, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);
  driver.begin();
  driver.toff(5);                 // enables the chopper; 0 = driver off
  driver.rms_current(currentMa);
  driver.microsteps(microsteps);
  driver.pwm_autoscale(true);
  driver.en_spreadCycle(false);   // stealthChop by default (quiet)
  driver.I_scale_analog(false);   // use UART current, ignore the VREF pot
#endif
#ifdef DRIVER_DRV8825
  pinMode(PIN_MS1, OUTPUT); pinMode(PIN_MS2, OUTPUT); pinMode(PIN_MS3, OUTPUT);
  pinMode(PIN_RST, OUTPUT); pinMode(PIN_SLP, OUTPUT);
  digitalWrite(PIN_RST, HIGH);    // out of reset
  digitalWrite(PIN_SLP, HIGH);    // awake
  driverSetMicrosteps(microsteps);
#endif
}

// ============================================================================
//  motion
// ============================================================================
void setDirection(int sign) {
  dirSign = (sign >= 0) ? +1 : -1;
  digitalWrite(PIN_DIR, dirSign > 0 ? HIGH : LOW);
}

void startMove(long steps, float sps) {          // steps<0 = reverse, 0 = continuous
  if (sps <= 0) return;
  setDirection(steps == 0 ? dirSign : (steps > 0 ? 1 : -1));
  stepsLeft = (steps == 0) ? -1 : labs(steps);
  tgtSps    = sps;
  running   = true;
  lastStepUs = micros();
  lastRampUs = lastStepUs;
}

void stopMove(bool hard) {
  if (hard) { running = false; curSps = 0; tgtSps = 0; stepsLeft = 0; }
  else      { tgtSps = 0; }                       // ramp down
}

void serviceSteps() {
  if (!running) return;
  uint32_t now = micros();

  // ---- ramp -----------------------------------------------------------
  float dt = (now - lastRampUs) * 1e-6f;
  if (dt > 0.0005f) {                             // update at ~2 kHz
    lastRampUs = now;
    float want = tgtSps;
    // decelerate in time to land on the target step for a finite move
    if (stepsLeft > 0) {
      float stopDist = (curSps * curSps) / (2.0f * accelSps2);
      if ((float)stepsLeft <= stopDist) want = 0.0f;
    }
    if      (curSps < want) curSps = min(want, curSps + accelSps2 * dt);
    else if (curSps > want) curSps = max(want, curSps - accelSps2 * dt);

    if (curSps <= 1.0f && want == 0.0f) {         // arrived / stopped
      curSps = 0;
      if (stepsLeft != -1) { running = false; stepsLeft = 0; return; }
      running = false; return;
    }
  }
  if (curSps < 1.0f) return;

  // ---- pulse ----------------------------------------------------------
  uint32_t interval = (uint32_t)(1000000.0f / curSps);
  if ((uint32_t)(now - lastStepUs) < interval) return;
  lastStepUs = now;

  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(PIN_STEP, LOW);

  position += dirSign;
  if (stepsLeft > 0) stepsLeft--;
}

// ============================================================================
//  command interpreter
// ============================================================================
String argStr(const String &s, int n) {           // n-th space-separated token
  int start = 0, count = 0;
  while (start < (int)s.length()) {
    int sp = s.indexOf(' ', start);
    if (sp < 0) sp = s.length();
    if (count == n) return s.substring(start, sp);
    count++; start = sp + 1;
  }
  return "";
}

void printHelp() {
  Serial.println(F("commands (case-insensitive):"));
  Serial.println(F("  PING                 identify + driver type"));
  Serial.println(F("  EN <0|1>             disable / enable the driver"));
  Serial.println(F("  DIR <0|1>            set direction (0=rev, 1=fwd)"));
  Serial.println(F("  MOVE <steps> [sps]   finite move, signed steps, ramped"));
  Serial.println(F("  REV <turns> [sps]    move N output-shaft-free motor revs (signed)"));
  Serial.println(F("  RUN <sps>            continuous, signed. STOP to ramp down"));
  Serial.println(F("  STOP                 ramped stop      | HALT = immediate"));
  Serial.println(F("  ACCEL <sps2>         ramp rate, steps/s^2"));
  Serial.println(F("  MICRO <n>            microstepping (TMC 1..256, DRV 1..32)"));
  Serial.println(F("  POS                  step counter     | ZERO = reset it"));
  Serial.println(F("  STATUS               full state dump"));
#ifdef DRIVER_TMC2209
  Serial.println(F("  CURRENT <mA>         RMS coil current via UART"));
  Serial.println(F("  SPREAD <0|1>         0 = stealthChop (quiet), 1 = spreadCycle"));
  Serial.println(F("  SG                   StallGuard result (lower = more load)"));
  Serial.println(F("  DIAG                 DRV_STATUS flags decoded"));
#endif
}

void printStatus() {
  Serial.print(F("driver=")); Serial.println(
#ifdef DRIVER_TMC2209
    F("TMC2209")
#else
    F("DRV8825")
#endif
  );
  Serial.print(F("enabled="));    Serial.println(enabled ? 1 : 0);
  Serial.print(F("microsteps=")); Serial.println(microsteps);
  Serial.print(F("position="));   Serial.println(position);
  Serial.print(F("running="));    Serial.println(running ? 1 : 0);
  Serial.print(F("cur_sps="));    Serial.println(curSps, 1);
  Serial.print(F("tgt_sps="));    Serial.println(tgtSps, 1);
  Serial.print(F("accel="));      Serial.println(accelSps2, 1);
  Serial.print(F("steps_left=")); Serial.println(stepsLeft);
  Serial.print(F("rev_per_s="));  Serial.println(curSps / (FULL_STEPS_PER_REV * microsteps), 3);
#ifdef DRIVER_TMC2209
  Serial.print(F("current_ma=")); Serial.println(currentMa);
  Serial.print(F("spreadcycle="));Serial.println(driver.en_spreadCycle() ? 1 : 0);
  Serial.print(F("tmc_version=0x")); Serial.println(driver.version(), HEX);
  Serial.print(F("tmc_ok="));     Serial.println(driver.test_connection() == 0 ? 1 : 0);
#endif
}

void handle(String line) {
  line.trim();
  if (!line.length()) return;
  String cmd = argStr(line, 0); cmd.toUpperCase();
  String a1  = argStr(line, 1);
  String a2  = argStr(line, 2);

  if (cmd == "HELP" || cmd == "?")      { printHelp(); }
  else if (cmd == "PING")               { Serial.println(F("stepper_test v1")); printStatus(); }
  else if (cmd == "STATUS")             { printStatus(); }
  else if (cmd == "EN")                 { driverEnable(a1.toInt() != 0); }
  else if (cmd == "DIR")                { setDirection(a1.toInt() ? 1 : -1); }
  else if (cmd == "ZERO")               { position = 0; }
  else if (cmd == "POS")                { Serial.print(F("position=")); Serial.println(position); }
  else if (cmd == "ACCEL")              { float v = a1.toFloat(); if (v > 0) accelSps2 = v;
                                          else { Serial.println(F("ERR accel must be > 0")); return; } }
  else if (cmd == "STOP")               { stopMove(false); }
  else if (cmd == "HALT")               { stopMove(true); }
  else if (cmd == "MOVE") {
    long st = a1.toInt();
    float sps = a2.length() ? a2.toFloat() : 1000.0f;
    if (!st)          { Serial.println(F("ERR steps must be non-zero")); return; }
    if (!enabled)     { Serial.println(F("ERR driver disabled - EN 1 first")); return; }
    startMove(st, sps);
  }
  else if (cmd == "REV") {
    float turns = a1.toFloat();
    float sps   = a2.length() ? a2.toFloat() : 1000.0f;
    if (turns == 0.0f) { Serial.println(F("ERR turns must be non-zero")); return; }
    if (!enabled)      { Serial.println(F("ERR driver disabled - EN 1 first")); return; }
    startMove((long)(turns * FULL_STEPS_PER_REV * microsteps), sps);
  }
  else if (cmd == "RUN") {
    float sps = a1.toFloat();
    if (sps == 0.0f) { Serial.println(F("ERR sps must be non-zero")); return; }
    if (!enabled)    { Serial.println(F("ERR driver disabled - EN 1 first")); return; }
    setDirection(sps > 0 ? 1 : -1);
    startMove(0, fabs(sps));
  }
  else if (cmd == "MICRO") {
    if (!driverSetMicrosteps((uint16_t)a1.toInt())) { Serial.println(F("ERR bad microstep value")); return; }
  }
#ifdef DRIVER_TMC2209
  else if (cmd == "CURRENT") {
    int ma = a1.toInt();
    if (ma < 100 || ma > 1500) { Serial.println(F("ERR current 100..1500 mA")); return; }
    currentMa = ma; driver.rms_current(ma);
  }
  else if (cmd == "SPREAD")  { driver.en_spreadCycle(a1.toInt() != 0); }
  else if (cmd == "SG")      { Serial.print(F("sg_result=")); Serial.println(driver.SG_RESULT()); }
  else if (cmd == "DIAG") {
    uint32_t s = driver.DRV_STATUS();
    Serial.print(F("drv_status=0x")); Serial.println(s, HEX);
    Serial.print(F("otpw="));  Serial.println(driver.otpw());   // over-temp warning
    Serial.print(F("ot="));    Serial.println(driver.ot());     // over-temp shutdown
    Serial.print(F("s2ga="));  Serial.println(driver.s2ga());   // short to gnd, coil A
    Serial.print(F("s2gb="));  Serial.println(driver.s2gb());
    Serial.print(F("ola="));   Serial.println(driver.ola());    // open load A (normal when stopped)
    Serial.print(F("olb="));   Serial.println(driver.olb());
    Serial.print(F("stst="));  Serial.println(driver.stst());   // standstill
    Serial.print(F("cs_actual=")); Serial.println(driver.cs_actual());
  }
#endif
  else { Serial.print(F("ERR unknown command: ")); Serial.println(cmd); return; }

  Serial.println(F("OK"));
}

// ============================================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) { }
  driverSetup();
  Serial.println();
  Serial.println(F("stepper_test v1 ready. Driver starts DISABLED - send: EN 1"));
  Serial.println(F("HELP for commands."));
#ifdef DRIVER_TMC2209
  if (driver.test_connection() != 0)
    Serial.println(F("WARN TMC2209 UART did not answer - check RX/TX swap, VIO, and GND."));
#endif
}

String rx;

void loop() {
  serviceSteps();
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { if (rx.length()) { handle(rx); rx = ""; } }
    else if (rx.length() < 96)  { rx += c; }
  }
}

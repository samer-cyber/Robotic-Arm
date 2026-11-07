#include "Protocol.h"
#include "Arm.h"
#include "TmcBus.h"
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

Protocol proto;

static bool ieq(const char* a, const char* b) { return strcasecmp(a, b) == 0; }

int parseJoint(const char* t) {
  if (ieq(t, "all")) return -1;
  for (uint8_t i = 0; i < JOINT_COUNT; i++) if (ieq(t, JCFG[i].name)) return i;
  char* end = nullptr;
  const long v = strtol(t, &end, 10);
  if (end == t || *end || v < 0 || v >= JOINT_COUNT) return -2;
  return (int)v;
}

// Say what is actually wrong, and what to do about it. An error whose detail
// describes a different problem is worse than no detail at all.
static const char* moveHint(ErrCode e, uint8_t j, float target) {
  static char buf[96];
  switch (e) {
    case E_DISABLED:
      snprintf(buf, sizeof(buf), "motors are off - run: enable 1");
      break;
    case E_ESTOP:
      snprintf(buf, sizeof(buf), "estop is latched - run: estop clear");
      break;
    case E_OUT_OF_RANGE:
      snprintf(buf, sizeof(buf), "%s target=%.2f limits=%.2f..%.2f",
               JCFG[j].name, target, JCFG[j].minDeg, JCFG[j].maxDeg);
      break;
    case E_NOT_ATTACHED:
      snprintf(buf, sizeof(buf), "%s is not present or failed to attach", JCFG[j].name);
      break;
    default:
      snprintf(buf, sizeof(buf), "%s", JCFG[j].name);
      break;
  }
  return buf;
}

void Protocol::begin(Stream& io) { s = &io; len = 0; }

void Protocol::replyOk() { s->println("ok"); }

void Protocol::replyErr(ErrCode e, const char* detail) {
  if (detail) s->printf("err %u %s %s\n", (unsigned)e, errName(e), detail);
  else        s->printf("err %u %s\n", (unsigned)e, errName(e));
}

void Protocol::emitEvent(const char* fmt, ...) {
  if (!s) return;
  char line[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  s->printf("! %s\n", line);
}

void Protocol::poll() {
  while (s->available()) {
    const char c = (char)s->read();
    if (c == '\r' || c == '\n') {
      buf[len] = 0;
      if (len) handleLine(buf);
      len = 0;
      continue;
    }
    if (c == 8 || c == 127) { if (len) len--; continue; }
    if (len < sizeof(buf) - 1) buf[len++] = c;
  }

  // Edge-triggered idle notification, so a host can wait on an event instead
  // of polling `state` in a loop.
  const bool moving = arm.isMoving();
  if (wasMoving && !moving) emitEvent("idle");
  wasMoving = moving;

  if (arm.lastEventText()[0]) {
    emitEvent("%s", arm.lastEventText());
    arm.clearEvent();
  }
}

void Protocol::handleLine(char* line) {
  char* argv[16];
  int argc = 0;
  for (char* t = strtok(line, " \t"); t && argc < 16; t = strtok(nullptr, " \t")) {
    if (t[0] == '#') break;
    argv[argc++] = t;
  }
  if (argc) dispatch(argc, argv);
}

// ---------------------------------------------------------------------------
void Protocol::dispatch(int argc, char** argv) {
  const char* c = argv[0];

  if (ieq(c, "help") || ieq(c, "?"))  { cmdHelp();  return; }
  if (ieq(c, "info"))                 { cmdInfo();  return; }
  if (ieq(c, "state"))                { cmdState(); return; }
  if (ieq(c, "config"))               { cmdConfig(argc, argv); return; }
  if (ieq(c, "set"))                  { cmdSet(argc, argv); return; }
  if (ieq(c, "pins") || ieq(c, "setpin")) { cmdPins(argc, argv); return; }
  if (ieq(c, "move"))                 { cmdMove(argc, argv, false); return; }
  if (ieq(c, "jog"))                  { cmdMove(argc, argv, true);  return; }
  if (ieq(c, "goto"))                 { cmdMoveAll(argc, argv); return; }
  if (ieq(c, "teach"))                { cmdTeach(argc, argv); return; }
  if (ieq(c, "driver") || ieq(c, "probe") || ieq(c, "bus")) { cmdDriver(argc, argv); return; }
  if (ieq(c, "diag"))                 { cmdDiag(argc, argv); return; }
  if (ieq(c, "grip"))                 { cmdGrip(argc, argv); return; }

  if (ieq(c, "enable")) {
    if (argc < 2) { replyErr(E_SYNTAX, "enable <0|1>"); return; }
    const bool on = atoi(argv[1]) != 0;
    if (on && arm.isEstopped()) { replyErr(E_ESTOP, "clear it first: estop clear"); return; }
    if (on) for (uint8_t i = 0; i < STEPPER_COUNT; i++) tmcBus.ensure(i);
    arm.setEnabled(on);
    s->printf("< enabled=%d\n", (int)arm.isEnabled());
    replyOk();
    return;
  }
  if (ieq(c, "stop"))  { arm.stopAll(false); replyOk(); return; }
  if (ieq(c, "estop")) {
    if (argc > 1 && ieq(argv[1], "clear")) {
      arm.clearEstop();
      s->println("< estop=0");
    } else {
      arm.emergencyStop();
      s->println("< estop=1");
    }
    replyOk();
    return;
  }
  if (ieq(c, "save"))     { replyOk(); configSave(); return; }
  if (ieq(c, "load"))     { configLoad(); tmcBus.applyAll(); replyOk(); return; }
  if (ieq(c, "defaults")) { configLoadDefaults(); tmcBus.applyAll(); replyOk(); return; }
  if (ieq(c, "reboot"))   { s->println("ok"); s->flush(); delay(80); ESP.restart(); return; }

  replyErr(E_UNKNOWN_CMD, c);
}

// ---------------------------------------------------------------------------
void Protocol::cmdHelp() {
  s->println("# --- system ---");
  s->println("#  info                       firmware, protocol, joint inventory");
  s->println("#  state                      positions, flags, driver health");
  s->println("#  config [joint]             all tunables");
  s->println("#  set <joint|all> <k> <v>    change one tunable");
  s->println("#  pins                       pin map + safety audit");
  s->println("#  setpin <joint> <step|dir|en> <gpio>");
  s->println("#  set <j> present 0|1        is this joint built? servos default off");
  s->println("#  save | load | defaults | reboot");
  s->println("# --- motion ---");
  s->println("#  enable <0|1>               energise the motors");
  s->println("#  move <joint> <deg> [feed]  absolute, feed 0..1");
  s->println("#  jog  <joint> <deg> [feed]  relative");
  s->println("#  goto <d1..d6> [feed]       all joints, arriving together");
  s->println("#  teach <joint|all> <deg>    redefine position, no motion");
  s->println("#  stop | estop | estop clear");
  s->println("# --- gripper (continuous-rotation servo on a tendon spool) ---");
  s->println("#  grip                       mode, wind estimate, tuning");
  s->println("#  grip close [power]         wind until stalled, then hold");
  s->println("#  grip open                  unwind for as long as it was wound, stop");
  s->println("#  grip stop                  neutral now (lets go of the motor)");
  s->println("#  grip run <-1..1> [ms]      raw drive for calibration, + = close");
  s->println("#  grip zero                  declare the hand open (wind = 0)");
  s->println("#  grip set <key> <value>     neutral_us span_us close_dir close_power");
  s->println("#                             close_ms hold open_ms max_run_ms continuous");
  s->println("# --- drivers ---");
  s->println("#  driver [joint]             decoded TMC2209 status");
  s->println("#  probe                      re-test the UART bus");
  s->println("#  bus echo | bus scan        raw bus diagnostics");
  s->println("# --- diagnostics ---");
  s->println("#  diag joint <joint>         is STEP/DIR/driver actually working");
  s->println("#  diag units <joint>         show every conversion factor");
  s->println("#  diag skip <joint> [deg] [cycles] [feed]   find lost steps");
  replyOk();
}

void Protocol::cmdInfo() {
  s->printf("< fw=%s version=%s proto=%u built=%s\n",
            FW_NAME, FW_VERSION, PROTO_VERSION, __DATE__);
  s->printf("< joints=%u steppers=%u servos=%u\n",
            JOINT_COUNT, STEPPER_COUNT, SERVO_COUNT);
  // Every joint is listed, built or not. Hiding the unbuilt ones made the arm
  // look like a 3-DOF machine, which is its own kind of lie. `present` says
  // whether the hardware exists; `attached` says whether the firmware got hold
  // of it. A host can show a greyed-out row instead of pretending nothing.
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    Joint* jt = arm.joint(i);
    s->printf("< joint=%s kind=%s present=%d attached=%d min=%.2f max=%.2f\n",
              JCFG[i].name, JCFG[i].kind == KIND_STEPPER ? "stepper" : "servo",
              (int)JCFG[i].present, jt && jt->attached() ? 1 : 0,
              JCFG[i].minDeg, JCFG[i].maxDeg);
  }
  replyOk();
}

void Protocol::cmdState() {
  s->printf("< enabled=%d moving=%d estop=%d\n",
            (int)arm.isEnabled(), (int)arm.isMoving(), (int)arm.isEstopped());
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    Joint* jt = arm.joint(i);
    s->printf("< %s pos=%.3f moving=%d present=%d attached=%d",
              JCFG[i].name, jt ? jt->position() : 0.0f,
              (int)(jt && jt->isMoving()), (int)JCFG[i].present,
              (int)(jt && jt->attached()));
    if (JCFG[i].kind == KIND_STEPPER && JCFG[i].present)
      s->printf(" driver=%s", tmcBus.online(i) ? "online" : "OFFLINE");
    if (i == JOINT_COUNT - 1 && arm.gripperJoint() && GCFG.continuous)
      s->printf(" grip=%s", arm.gripperJoint()->modeName());
    s->println();
  }
  replyOk();
}

// ---------------------------------------------------------------------------
//  Gripper. GRIP stays a joint (so `goto` and saved poses still carry it), but
//  a continuous servo needs verbs, not angles - hence its own command.
// ---------------------------------------------------------------------------
void Protocol::gripReport() {
  GripperJoint* g = arm.gripperJoint();
  s->printf("< grip mode=%s wind=%.1f speed=%.2f continuous=%d\n",
            g ? g->modeName() : "absent", g ? g->windPct() : 0.0f,
            g ? g->speed() : 0.0f, (int)GCFG.continuous);
  s->printf("< grip neutral_us=%u span_us=%u close_dir=%d close_power=%.2f "
            "close_ms=%u hold=%.2f open_ms=%u max_run_ms=%u\n",
            GCFG.neutralUs, GCFG.spanUs, (int)GCFG.closeDir, GCFG.closePower,
            GCFG.closeMs, GCFG.holdPower, GCFG.openMs, GCFG.maxRunMs);
}

void Protocol::cmdGrip(int argc, char** argv) {
  GripperJoint* g = arm.gripperJoint();
  if (argc < 2) { gripReport(); replyOk(); return; }
  const char* sub = argv[1];

  if (ieq(sub, "set")) {
    if (argc < 4) { replyErr(E_SYNTAX, "grip set <key> <value>"); return; }
    const char* k = argv[2];
    const float v = atof(argv[3]);
    if      (ieq(k, "neutral_us"))  GCFG.neutralUs  = (uint16_t)constrain(v, 500.0f, 2500.0f);
    else if (ieq(k, "span_us"))     GCFG.spanUs     = (uint16_t)constrain(v, 0.0f, 1000.0f);
    else if (ieq(k, "close_dir"))   GCFG.closeDir   = (v < 0) ? -1 : 1;
    else if (ieq(k, "close_power")) GCFG.closePower = constrain(v, 0.05f, 1.0f);
    else if (ieq(k, "close_ms"))    GCFG.closeMs    = (uint16_t)constrain(v, 50.0f, 10000.0f);
    else if (ieq(k, "hold"))        GCFG.holdPower  = constrain(v, 0.0f, 1.0f);
    else if (ieq(k, "open_ms"))     GCFG.openMs     = (uint16_t)constrain(v, 50.0f, 10000.0f);
    else if (ieq(k, "max_run_ms"))  GCFG.maxRunMs   = (uint16_t)constrain(v, 100.0f, 20000.0f);
    else if (ieq(k, "continuous"))  GCFG.continuous = v != 0;
    else { replyErr(E_BAD_VALUE, k); return; }
    s->println("# stored. `save` to keep it across a reboot");
    gripReport();
    replyOk();
    return;
  }

  if (!g) { replyErr(E_NOT_ATTACHED, "GRIP is not present or failed to attach"); return; }
  if (ieq(sub, "zero")) { g->redefinePosition(0.0f); gripReport(); replyOk(); return; }
  if (ieq(sub, "stop")) { g->halt(); gripReport(); replyOk(); return; }

  if (arm.isEstopped()) { replyErr(E_ESTOP, "estop is latched - run: estop clear"); return; }
  if (!arm.isEnabled()) { replyErr(E_DISABLED, "motors are off - run: enable 1"); return; }

  ErrCode e;
  if      (ieq(sub, "close")) e = g->close(argc > 2 ? atof(argv[2]) : 0.0f);
  else if (ieq(sub, "open"))  e = g->open();
  else if (ieq(sub, "run")) {
    if (argc < 3) { replyErr(E_SYNTAX, "grip run <-1..1> [ms]"); return; }
    e = g->run(atof(argv[2]), argc > 3 ? (uint32_t)atol(argv[3]) : 300);
  }
  else { replyErr(E_SYNTAX, "grip [close|open|stop|run|zero|set]"); return; }

  if (e != E_OK) { replyErr(e, "GRIP"); return; }
  gripReport();
  replyOk();
}

void Protocol::cmdConfig(int argc, char** argv) {
  int sel = (argc > 1) ? parseJoint(argv[1]) : -1;
  if (sel == -2) { replyErr(E_BAD_JOINT, argv[1]); return; }
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (sel >= 0 && (int)i != sel) continue;
    const JointConfig& c = JCFG[i];
    s->printf("< %s kind=%s gear=%.3f usteps=%u steps_per_rev=%.0f invert=%d\n",
              c.name, c.kind == KIND_STEPPER ? "stepper" : "servo",
              c.gearRatio, c.microsteps, c.motorStepsPerRev, (int)c.invert);
    s->printf("< %s min=%.2f max=%.2f vel=%.1f acc=%.1f\n",
              c.name, c.minDeg, c.maxDeg, c.maxVelDeg, c.maxAccDeg);
    if (c.kind == KIND_STEPPER) {
      StepperJoint* sj = (StepperJoint*)arm.joint(i);
      s->printf("< %s current=%u hold=%.2f chopper=%s intpol=%d steps_per_deg=%.4f\n",
                c.name, c.runCurrent_mA, c.holdFactor,
                c.stealthChop ? "stealth" : "spread", (int)c.interpolate,
                sj ? sj->stepsPerDegree() : 0.0f);
    } else {
      s->printf("< %s servo_us=%u..%u servo_deg=%.1f..%.1f\n",
                c.name, c.servoMinUs, c.servoMaxUs, c.servoMinDeg, c.servoMaxDeg);
    }
  }
  replyOk();
}

void Protocol::cmdSet(int argc, char** argv) {
  if (argc < 4) { replyErr(E_SYNTAX, "set <joint|all> <key> <value>"); return; }
  const int sel = parseJoint(argv[1]);
  if (sel == -2) { replyErr(E_BAD_JOINT, argv[1]); return; }
  const char* k = argv[2];
  const float v = atof(argv[3]);

  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (sel >= 0 && (int)i != sel) continue;
    JointConfig& c = JCFG[i];
    Joint* jt = arm.joint(i);
    const float held = jt ? jt->position() : 0.0f;
    bool rescale = false, redriver = false;

    if      (ieq(k, "gear"))     { c.gearRatio = v;                rescale = true; }
    else if (ieq(k, "usteps"))   { c.microsteps = (uint16_t)v;      rescale = redriver = true; }
    else if (ieq(k, "steps_per_rev")) { c.motorStepsPerRev = v;     rescale = true; }
    else if (ieq(k, "present"))  { c.present = v != 0;
                                   s->println("# takes effect after: save, then reboot"); }
    else if (ieq(k, "invert"))   { c.invert = v != 0;               redriver = true; }
    else if (ieq(k, "min"))      { c.minDeg = v; }
    else if (ieq(k, "max"))      { c.maxDeg = v; }
    else if (ieq(k, "vel"))      { c.maxVelDeg = v; }
    else if (ieq(k, "acc"))      { c.maxAccDeg = v; }
    else if (ieq(k, "current"))  { c.runCurrent_mA = (uint16_t)v;   redriver = true; }
    else if (ieq(k, "hold"))     { c.holdFactor = v;                redriver = true; }
    else if (ieq(k, "chopper"))  { c.stealthChop = ieq(argv[3], "stealth") || v == 1; redriver = true; }
    else if (ieq(k, "intpol"))   { c.interpolate = v != 0;          redriver = true; }
    else if (ieq(k, "servo_min_us")) { c.servoMinUs = (uint16_t)v; }
    else if (ieq(k, "servo_max_us")) { c.servoMaxUs = (uint16_t)v; }
    else if (ieq(k, "servo_min_deg")) { c.servoMinDeg = v; }
    else if (ieq(k, "servo_max_deg")) { c.servoMaxDeg = v; }
    else { replyErr(E_BAD_VALUE, k); return; }

    // Changing a conversion factor must not teleport the joint: keep the angle
    // and recompute the step count behind it.
    if (rescale && jt) jt->redefinePosition(held);
    if (redriver && i < STEPPER_COUNT) {
      if (tmcBus.ensure(i)) tmcBus.applyConfig(i);
      else s->printf("# !! %s driver offline - stored but NOT written to the chip\n",
                     JCFG[i].name);
    }
  }
  replyOk();
}

void Protocol::cmdPins(int argc, char** argv) {
  if (argc >= 4 && ieq(argv[0], "setpin")) {
    const int a = parseJoint(argv[1]);
    if (a < 0) { replyErr(E_BAD_JOINT, argv[1]); return; }
    if (ieq(argv[2], "dir")) {
      replyErr(E_UNSUPPORTED, "DIR pins are unused - direction is the TMC2209 shaft bit over UART. see: diag dir");
      return;
    }
    const int g = atoi(argv[3]);
    const char* why = pinUnusableReason(g);
    if (why) { replyErr(E_BAD_PIN, why); return; }
    const int owner = pinOwner(a, g);
    if (owner >= 0) { replyErr(E_BAD_PIN, JCFG[owner].name); return; }

    if      (ieq(argv[2], "step")) JPIN[a].step   = (uint8_t)g;
    else if (ieq(argv[2], "en"))   JPIN[a].enable = (uint8_t)g;
    else if (ieq(argv[2], "addr")) JPIN[a].tmcAddr= (uint8_t)g;
    else { replyErr(E_SYNTAX, "step|en|addr"); return; }
    s->println("# stored. takes effect after: save, then reboot");
    replyOk();
    return;
  }

  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (JCFG[i].kind == KIND_STEPPER)
      s->printf("< %s step=%u dir=uart-shaft en=%u addr=%u\n",
                JCFG[i].name, JPIN[i].step, JPIN[i].enable, JPIN[i].tmcAddr);
    else
      s->printf("< %s pwm=%u\n", JCFG[i].name, JPIN[i].step);
  }
  s->printf("< uart tx=%u rx=%u\n", TMC_UART_TX, TMC_UART_RX);
  if (pinAudit(*s)) s->println("# audit clean");
  replyOk();
}

void Protocol::cmdMove(int argc, char** argv, bool relative) {
  if (argc < 3) { replyErr(E_SYNTAX, "<joint> <deg> [feed]"); return; }
  const int a = parseJoint(argv[1]);
  if (a < 0) { replyErr(E_BAD_JOINT, argv[1]); return; }
  Joint* jt = arm.joint(a);
  if (!jt) { replyErr(E_BAD_JOINT, argv[1]); return; }

  const float d = atof(argv[2]);
  const float feed = (argc > 3) ? atof(argv[3]) : 1.0f;
  const float target = relative ? jt->position() + d : d;

  const ErrCode e = arm.moveJoint(a, target, feed);
  if (e != E_OK) { replyErr(e, moveHint(e, a, target)); return; }
  s->printf("< target %s=%.3f\n", JCFG[a].name, target);
  replyOk();
}

void Protocol::cmdMoveAll(int argc, char** argv) {
  float targets[JOINT_COUNT];
  for (uint8_t i = 0; i < JOINT_COUNT; i++) targets[i] = NAN;

  int n = 0;
  for (int i = 1; i < argc && n < JOINT_COUNT; i++) {
    if (ieq(argv[i], "-")) { n++; continue; }     // "-" = leave this joint alone
    targets[n++] = atof(argv[i]);
  }
  const float feed = (argc > 1 + n) ? atof(argv[1 + n]) : 1.0f;
  if (n == 0) { replyErr(E_SYNTAX, "goto <d1> .. <dN> [feed]"); return; }

  uint8_t blame = 0xFF;
  const ErrCode e = arm.moveAll(targets, feed, &blame);
  if (e != E_OK) {
    replyErr(e, blame < JOINT_COUNT ? moveHint(e, blame, targets[blame])
                                    : "no joint named");
    return;
  }
  replyOk();
}

void Protocol::cmdTeach(int argc, char** argv) {
  if (argc < 3) { replyErr(E_SYNTAX, "teach <joint|all> <deg>"); return; }
  const int sel = parseJoint(argv[1]);
  if (sel == -2) { replyErr(E_BAD_JOINT, argv[1]); return; }
  const float v = atof(argv[2]);
  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (sel >= 0 && (int)i != sel) continue;
    Joint* jt = arm.joint(i);
    if (jt) jt->redefinePosition(v);
  }
  replyOk();
}

void Protocol::cmdDriver(int argc, char** argv) {
  if (ieq(argv[0], "probe")) {
    for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
      const bool up = tmcBus.probe(i);
      if (up) tmcBus.applyConfig(i);
      s->printf("< %s driver=%s\n", JCFG[i].name, up ? "online" : "offline");
    }
    replyOk();
    return;
  }
  if (ieq(argv[0], "bus")) {
    if (argc > 1 && ieq(argv[1], "echo")) { tmcBus.echoTest(*s); replyOk(); return; }
    if (argc > 1 && ieq(argv[1], "scan")) { tmcBus.addressScan(*s); replyOk(); return; }
    replyErr(E_SYNTAX, "bus echo|scan");
    return;
  }
  const int sel = (argc > 1) ? parseJoint(argv[1]) : -1;
  if (sel == -2) { replyErr(E_BAD_JOINT, argv[1]); return; }
  for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
    if (sel >= 0 && (int)i != sel) continue;
    tmcBus.report(*s, i);
  }
  replyOk();
}

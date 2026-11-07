#include "console.h"
#include "servos.h"
#include "drivers.h"
#include "motion.h"
#include "kinematics.h"
#include "tests.h"
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <math.h>

Console console;

// ---------------------------------------------------------------- utils ----
bool ieq(const char *a, const char *b) { return strcasecmp(a, b) == 0; }

// Resolve an axis token: "0".."5", "J1".."J6", or "all" (-1). -2 = garbage.
int axisArg(const char *t) {
  if (ieq(t, "all")) return -1;
  for (uint8_t i = 0; i < N_AXES; i++) if (ieq(t, AX[i].name)) return i;
  char *end = nullptr;
  long v = strtol(t, &end, 10);
  if (end == t || *end) return -2;
  if (v < 0 || v >= N_AXES) return -2;
  return (int)v;
}

#define FOR_AXES(sel, i) for (uint8_t i = (sel < 0 ? 0 : sel); \
                              i < (sel < 0 ? N_AXES : sel + 1); i++)

void Console::begin(Stream &s) { s_ = &s; len = 0; }

void Console::poll() {
  while (s_->available()) {
    const char c = (char)s_->read();

    // Accept CR, LF or CRLF as end-of-line. Terminals disagree about which one
    // Enter sends, and the disagreement is invisible: you type, you hit Enter,
    // and absolutely nothing happens. Take all three.
    if (c == '\r' || c == '\n') {
      buf[len] = 0;
      if (len) {
        char *argv[12];
        int argc = 0;
        for (char *t = strtok(buf, " \t"); t && argc < 12; t = strtok(nullptr, " \t")) {
          if (t[0] == '#') break;      // rest of the line is a comment
          argv[argc++] = t;
        }
        if (argc) dispatch(argc, argv);
      }
      len = 0;
      continue;
    }
    if (c == 8 || c == 127) { if (len) len--; continue; }   // backspace
    if (len < sizeof(buf) - 1) buf[len++] = c;
  }
}

// ----------------------------------------------------------------- help ----
void Console::help() {
  Stream &o = *s_;
  o.println("# --- system -------------------------------------------------");
  o.println("#  ?                       this help");
  o.println("#  ver                     firmware / build info");
  o.println("#  pinmap                  which GPIO drives what, with warnings");
  o.println("#  setpin <ax> <step|dir|en> <gpio>   match firmware to real wiring");
  o.println("#  reboot                  restart the board");
  o.println("#  stat                    joints, tool pose, driver health");
  o.println("#  cfg                     dump all axis config + geometry");
  o.println("#  set <ax|all> <k> <v>    edit config (cfg lists the key names)");
  o.println("#  geo [<k> <v>]           show / edit link geometry, mm");
  o.println("#  save | load | defaults  NVS persistence");
  o.println("# --- servos (J4/J5/J6 + gripper, OPEN LOOP) -----------------");
  o.println("#  sv                      list servos, angles, pulses, limits");
  o.println("#  sv <J4|J5|J6|grip> <deg>   command one. 0 = middle of travel");
  o.println("#  grip <0-100|open|close> jaw");
  o.println("#  svset <id> <k> <v>      pin range minus maxus min max trim inv park");
  o.println("#  svpark | svoff | svattach   park / go limp / resume pulses");
  o.println("#  desk [<z> [margin]]     work-surface floor + live clearance");
  o.println("#  mon [ms]                stream live telemetry, key to stop");
  o.println("# --- power and motion ---------------------------------------");
  o.println("#  en <0|1>                driver ENable, motors go stiff");
  o.println("#  mv  <ax> <deg> [feed]   relative move, feed = 0..1 of max vel");
  o.println("#  mva <ax> <deg> [feed]   absolute move");
  o.println("#  j <q1> <q2> <q3> [feed] coordinated move, all axes arrive together");
  o.println("#  hz <ax> <steps/s>       free-run at a raw step rate");
  o.println("#  home <ax|all> [sg]      home on limit switch, or sg = StallGuard");
  o.println("#  zero [ax|all]           call the current position zero");
  o.println("#  pos <ax|all> <deg>      call the current position <deg>, no motion");
  o.println("#  stop | estop            ramp down | instant stop and disable");
  o.println("#  wait                    block until idle, host sync point");
  o.println("# --- kinematics ---------------------------------------------");
  o.println("#  fk [q1 q2 q3]           joints -> XYZ, no args = where I am");
  o.println("#  ik <x> <y> <z> [up|dn]  solve only, print joints, do not move");
  o.println("#  gt <x> <y> <z> [feed]   solve and go there");
  o.println("#  line <x> <y> <z> [seg] [feed]   segmented straight-line move");
  o.println("# --- TMC2209 ------------------------------------------------");
  o.println("#  tmc [ax]                full decoded driver report");
  o.println("#  probe                   re-test the UART link to every driver");
  o.println("#  uart loop               echo test - proves TX/RX/wiring, no driver needed");
  o.println("#  uart scan               raw datagram to addr 0..3, dumps the wire in hex");
  o.println("#  uart pins               which pins and which UART peripheral");
  o.println("#  dir <ax> <0|1>          hold the DIR line so you can meter it");
  o.println("#  stp <ax> <n> [ms]       direct slow STEP pulses, bypasses everything");
  o.println("#  cur  <ax|all> <mA> [holdmult]");
  o.println("#  ms   <ax|all> <1..256>  microsteps");
  o.println("#  sc   <ax|all> <0|1>     1 = StealthChop, 0 = SpreadCycle");
  o.println("#  tpwm <ax|all> <tstep>   auto StealthChop -> SpreadCycle changeover");
  o.println("#  sg   <ax|all> <0..255>  StallGuard threshold");
  o.println("#  sgr  [ax]               live SG_RESULT stream, key to stop");
  o.println("#  cool <ax|all> <semin> <semax>   CoolStep, semin 0 = off");
  o.println("# --- tests --------------------------------------------------");
  o.println("#  t pins <ax>             is STEP/DIR/EN actually doing anything?");
  o.println("#  t ik [n]                IK->FK round-trip error, no motion");
  o.println("#  t reach [n]             how much of the workspace is reachable");
  o.println("#  t speed <ax> [maxHz] [stepHz] [dwellms]   ramp until it stalls");
  o.println("#  t sweep <ax> <cycles> <degA> <degB> [feed]");
  o.println("#  t rep <ax> <cycles>     homing repeatability");
  o.println("ok");
}

// --------------------------------------------------------------- status ----
void Console::status() {
  Stream &o = *s_;
  float q[MAX_AXES];
  motion.jointsDeg(q);
  Pose p;
  fkPose(q, p);

  o.printf("# state    %s, %s\n", motion.isEnabled() ? "ENABLED" : "disabled",
           motion.moving() ? "MOVING" : "idle");
  for (uint8_t i = 0; i < N_AXES; i++) {
    o.printf("# %s       %8.3f deg  %8ld steps  vmax %.1f deg/s  %s%s\n",
             AX[i].name, q[i],
             motion.stepper(i) ? motion.stepper(i)->getCurrentPosition() : 0L,
             AX[i].maxVel,
             motion.limitHit(i) ? "[LIMIT] " : "",
             drivers.online[i] ? "" : "[DRIVER OFFLINE]");
  }
  o.printf("# tool     x %.2f  y %.2f  z %.2f  mm\n", p.x, p.y, p.z);
  if (motion.lastEvent()[0]) {
    o.printf("# event    %s\n", motion.lastEvent());
    motion.clearEvent();
  }
  o.println("ok");
}

// --------------------------------------------------------------- config ----
void Console::dumpCfg() {
  Stream &o = *s_;
  o.println("# key        J1        J2        J3        <- set <ax|all> <key> <val>");
  o.printf("# steps    "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10.0f", AX[i].fullSteps);  o.println();
  o.printf("# us       "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10u",   AX[i].microsteps); o.println();
  o.printf("# gear     "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10.3f", AX[i].gear);       o.println();
  o.printf("# inv      "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10d",   (int)AX[i].invert);o.println();
  o.printf("# min      "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10.2f", AX[i].minDeg);     o.println();
  o.printf("# max      "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10.2f", AX[i].maxDeg);     o.println();
  o.printf("# hdeg     "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10.2f", AX[i].homeDeg);    o.println();
  o.printf("# hdir     "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10d",   (int)AX[i].homeDir);o.println();
  o.printf("# vel      "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10.1f", AX[i].maxVel);     o.println();
  o.printf("# acc      "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10.1f", AX[i].maxAcc);     o.println();
  o.printf("# hvel     "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10.1f", AX[i].homeVel);    o.println();
  o.printf("# ma       "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10u",   AX[i].runmA);      o.println();
  o.printf("# hold     "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10.2f", AX[i].holdMult);   o.println();
  o.printf("# sc       "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10d",   (int)AX[i].stealth);o.println();
  o.printf("# intpol   "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10d",   (int)AX[i].intpol);o.println();
  o.printf("# sg       "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10u",   AX[i].sgthrs);     o.println();
  o.printf("# tcool    "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10lu", (unsigned long)AX[i].tcoolthrs); o.println();
  o.printf("# tpwm     "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10lu", (unsigned long)AX[i].tpwmthrs);  o.println();
  o.printf("# semin    "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10u",   AX[i].semin);      o.println();
  o.printf("# semax    "); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10u",   AX[i].semax);      o.println();
  o.printf("# steps/deg"); for (uint8_t i=0;i<N_AXES;i++) o.printf("%-10.3f", motion.stepsPerDeg(i)); o.println();
  o.printf("# geo      baseHeight %.1f  baseOffset %.1f  l1 %.1f  l2 %.1f  elbow %s\n",
           GEO.baseHeight, GEO.baseOffset, GEO.l1, GEO.l2, ELBOW_UP ? "up" : "down");
  o.printf("# reach    %.1f .. %.1f mm measured from the shoulder\n", reachMin(), reachMax());
  o.println("ok");
}

bool Console::cmdSet(int argc, char **argv) {
  Stream &o = *s_;
  if (argc < 4) { o.println("err usage: set <ax|all> <key> <value>"); return false; }
  const int sel = axisArg(argv[1]);
  if (sel == -2) { o.printf("err bad axis %s\n", argv[1]); return false; }
  const char *k = argv[2];
  const float v = atof(argv[3]);

  FOR_AXES(sel, i) {
    AxisCfg &a = AX[i];
    const float held = motion.jointDeg(i);   // keep the angle across rescales
    bool rescale = false, redriver = false;

    if      (ieq(k,"steps"))  { a.fullSteps  = v;           rescale  = true; }
    else if (ieq(k,"us"))     { a.microsteps = (uint16_t)v; rescale  = redriver = true; }
    else if (ieq(k,"gear"))   { a.gear       = v;           rescale  = true; }
    else if (ieq(k,"inv"))    { a.invert     = v != 0;      redriver = true; }
    else if (ieq(k,"min"))    { a.minDeg     = v; }
    else if (ieq(k,"max"))    { a.maxDeg     = v; }
    else if (ieq(k,"hdeg"))   { a.homeDeg    = v; }
    else if (ieq(k,"hdir"))   { a.homeDir    = (v < 0) ? -1 : 1; }
    else if (ieq(k,"vel"))    { a.maxVel     = v; }
    else if (ieq(k,"acc"))    { a.maxAcc     = v; }
    else if (ieq(k,"hvel"))   { a.homeVel    = v; }
    else if (ieq(k,"ma"))     { a.runmA      = (uint16_t)v; redriver = true; }
    else if (ieq(k,"hold"))   { a.holdMult   = v;           redriver = true; }
    else if (ieq(k,"sc"))     { a.stealth    = v != 0;      redriver = true; }
    else if (ieq(k,"intpol")) { a.intpol     = v != 0;      redriver = true; }
    else if (ieq(k,"sg"))     { a.sgthrs     = (uint8_t)v;  redriver = true; }
    else if (ieq(k,"tcool"))  { a.tcoolthrs  = (uint32_t)v; redriver = true; }
    else if (ieq(k,"tpwm"))   { a.tpwmthrs   = (uint32_t)v; redriver = true; }
    else if (ieq(k,"semin"))  { a.semin      = (uint8_t)v;  redriver = true; }
    else if (ieq(k,"semax"))  { a.semax      = (uint8_t)v;  redriver = true; }
    else { o.printf("err unknown key %s, run cfg for the list\n", k); return false; }

    if (rescale) motion.setJointDeg(i, held);
    if (redriver) {
      if (drivers.ensure(i)) drivers.apply(i);
      else o.printf("# WARNING %s driver is offline. Value stored, but NOT "
                    "written to the chip. Run: probe\n", AX[i].name);
    }
  }
  o.println("ok");
  return true;
}

bool Console::cmdGeo(int argc, char **argv) {
  Stream &o = *s_;
  if (argc == 1) {
    o.printf("# baseHeight %.2f  baseOffset %.2f  l1 %.2f  l2 %.2f  elbow %s\n",
             GEO.baseHeight, GEO.baseOffset, GEO.l1, GEO.l2, ELBOW_UP ? "up" : "down");
    o.println("ok");
    return true;
  }
  if (argc < 3) { o.println("err usage: geo <key> <value>"); return false; }
  const char *k = argv[1];
  const float v = atof(argv[2]);
  if      (ieq(k,"baseheight") || ieq(k,"bh")) GEO.baseHeight = v;
  else if (ieq(k,"baseoffset") || ieq(k,"bo")) GEO.baseOffset = v;
  else if (ieq(k,"l1"))                        GEO.l1 = v;
  else if (ieq(k,"l2"))                        GEO.l2 = v;
  else if (ieq(k,"elbow"))                     ELBOW_UP = ieq(argv[2], "up") || v != 0;
  else { o.printf("err unknown geo key %s\n", k); return false; }
  o.println("ok");
  return true;
}

void Console::monitor(uint32_t periodMs) {
  Stream &o = *s_;
  o.println("# streaming, press any key to stop");
  while (true) {
    if (s_->available()) { while (s_->available()) s_->read(); break; }
    float q[MAX_AXES];
    motion.jointsDeg(q);
    Pose p;
    fkPose(q, p);
    o.printf("t=%lu ", (unsigned long)millis());
    for (uint8_t i = 0; i < N_AXES; i++) o.printf("%s=%.2f ", AX[i].name, q[i]);
    o.printf("| xyz %.1f %.1f %.1f |", p.x, p.y, p.z);
    for (uint8_t i = 0; i < N_AXES; i++) {
      if (!drivers.online[i]) { o.printf(" %s:off", AX[i].name); continue; }
      o.printf(" %s:sg=%u,cs=%u%s", AX[i].name,
               drivers.d(i)->SG_RESULT(), drivers.d(i)->cs_actual(),
               drivers.d(i)->otpw() ? ",OTPW!" : "");
    }
    o.println();
    const uint32_t t0 = millis();
    while (millis() - t0 < periodMs) { motion.poll(); delay(1); }
  }
  o.println("ok");
}

// ------------------------------------------------------------- dispatch ----
void Console::dispatch(int argc, char **argv) {
  Stream &o = *s_;
  const char *c = argv[0];
  char err[96] = {0};

  // ---- system ----
  if (ieq(c, "?") || ieq(c, "help")) { help(); return; }

  if (ieq(c, "ver")) {
    o.printf("# %s %s  built %s %s\n", FW_NAME, FW_VERSION, __DATE__, __TIME__);
    o.printf("# axes %d of %d   host baud %d   tmc baud %d   Rsense %.3f\n",
             N_AXES, MAX_AXES, HOST_BAUD, TMC_BAUD, R_SENSE);
    o.println("ok");
    return;
  }
  if (ieq(c, "stat")) { status(); return; }
  if (ieq(c, "cfg"))  { dumpCfg(); return; }
  if (ieq(c, "set"))  { cmdSet(argc, argv); return; }
  if (ieq(c, "geo"))  { cmdGeo(argc, argv); return; }
  if (ieq(c, "save")) { o.println(cfgSave() ? "ok" : "err nvs write failed"); return; }
  if (ieq(c, "load")) {
    if (!cfgLoad()) { o.println("err nothing saved"); return; }
    drivers.applyAll();
    o.println("ok");
    return;
  }
  if (ieq(c, "defaults")) {
    cfgLoadDefaults();
    drivers.applyAll();
    o.println("ok");
    return;
  }
  if (ieq(c, "mon")) { monitor(argc > 1 ? (uint32_t)atol(argv[1]) : 200); return; }

  // ---- power and motion ----
  if (ieq(c, "en")) {
    if (argc < 2) { o.println("err usage: en <0|1>"); return; }
    const bool on = atoi(argv[1]) != 0;
    if (on) {
      // Energising with an unconfigured chip means default microstepping and
      // VREF-trimpot current: the motor moves a fraction of what you asked for,
      // very weakly. Catch it here rather than let the user wonder.
      for (uint8_t i = 0; i < N_AXES; i++)
        if (!drivers.ensure(i))
          o.printf("# WARNING %s driver offline - it is running on power-up "
                   "defaults, not your settings\n", AX[i].name);
    }
    motion.enable(on);
    o.printf("# drivers %s\n", motion.isEnabled() ? "ENABLED" : "disabled");
    o.println("ok");
    return;
  }
  if (ieq(c, "estop")) {
    motion.stopAll(true);
    motion.enable(false);
    o.println("# EMERGENCY STOP, drivers disabled");
    o.println("ok");
    return;
  }
  if (ieq(c, "stop")) { motion.stopAll(false); o.println("ok"); return; }

  if (ieq(c, "wait")) {
    motion.waitIdle(o);
    o.println("ok");
    return;
  }
  if (ieq(c, "zero")) {
    const int sel = (argc > 1) ? axisArg(argv[1]) : -1;
    if (sel == -2) { o.printf("err bad axis %s\n", argv[1]); return; }
    FOR_AXES(sel, i) motion.setJointDeg(i, 0.0f);
    o.println("ok");
    return;
  }
  // Declare the CURRENT physical pose to be some angle. Nothing moves. This is
  // how you establish a reference without limit switches: park the arm
  // somewhere you can measure, then tell the firmware what it is looking at.
  if (ieq(c, "pos")) {
    if (argc < 3) { o.println("err usage: pos <ax|all> <deg>"); return; }
    const int sel = axisArg(argv[1]);
    if (sel == -2) { o.printf("err bad axis %s\n", argv[1]); return; }
    const float deg = atof(argv[2]);
    FOR_AXES(sel, i) {
      if (deg < AX[i].minDeg - 1e-3f || deg > AX[i].maxDeg + 1e-3f) {
        o.printf("err %.2f is outside %s limits [%.2f, %.2f] - widen them first\n",
                 deg, AX[i].name, AX[i].minDeg, AX[i].maxDeg);
        return;
      }
      motion.setJointDeg(i, deg);
    }
    o.println("ok");
    return;
  }
  if (ieq(c, "mv") || ieq(c, "mva")) {
    if (argc < 3) { o.printf("err usage: %s <ax> <deg> [feed]\n", c); return; }
    const int a = axisArg(argv[1]);
    if (a < 0) { o.println("err need a single axis"); return; }
    const float deg  = atof(argv[2]);
    const float feed = (argc > 3) ? atof(argv[3]) : 1.0f;
    const float target = ieq(c, "mv") ? motion.jointDeg(a) + deg : deg;
    if (!motion.moveAxisSteps(a, motion.degToSteps(a, target), feed, err, sizeof(err)))
      { o.printf("err %s\n", err); return; }
    o.println("ok");
    return;
  }
  if (ieq(c, "j")) {
    if (argc < 1 + N_AXES) { o.printf("err usage: j <q1..q%d> [feed]\n", N_AXES); return; }
    float t[MAX_AXES];
    for (uint8_t i = 0; i < N_AXES; i++) t[i] = atof(argv[1 + i]);
    const float feed = (argc > 1 + N_AXES) ? atof(argv[1 + N_AXES]) : 1.0f;
    if (!motion.moveJointsDeg(t, feed, err, sizeof(err))) { o.printf("err %s\n", err); return; }
    o.println("ok");
    return;
  }
  if (ieq(c, "hz")) {
    if (argc < 3) { o.println("err usage: hz <ax> <steps/s, sign = direction>"); return; }
    const int a = axisArg(argv[1]);
    if (a < 0) { o.println("err need a single axis"); return; }
    const float hz = atof(argv[2]);
    if (!motion.runContinuous(a, hz < 0 ? -1 : 1, fabsf(hz), err, sizeof(err)))
      { o.printf("err %s\n", err); return; }
    o.println("# free-running, use stop");
    o.println("ok");
    return;
  }
  if (ieq(c, "home")) {
    if (argc < 2) { o.println("err usage: home <ax|all> [sg]"); return; }
    const int sel = axisArg(argv[1]);
    if (sel == -2) { o.printf("err bad axis %s\n", argv[1]); return; }
    const bool sg = (argc > 2) && ieq(argv[2], "sg");
    bool allOk = true;
    FOR_AXES(sel, i) if (!motion.home(i, sg, o)) { allOk = false; break; }
    o.println(allOk ? "ok" : "err homing failed");
    return;
  }

  // ---- kinematics ----
  if (ieq(c, "fk")) {
    float q[MAX_AXES];
    if (argc >= 4) { for (uint8_t i = 0; i < 3; i++) q[i] = atof(argv[1 + i]); }
    else           { motion.jointsDeg(q); }
    Pose p;
    fkPose(q, p);
    o.printf("# q %.3f %.3f %.3f  ->  xyz %.3f %.3f %.3f\n",
             q[0], q[1], q[2], p.x, p.y, p.z);
    o.println("ok");
    return;
  }
  if (ieq(c, "ik") || ieq(c, "gt")) {
    if (argc < 4) { o.printf("err usage: %s <x> <y> <z> [up|dn|feed]\n", c); return; }
    Pose p = { (float)atof(argv[1]), (float)atof(argv[2]), (float)atof(argv[3]) };
    bool up = ELBOW_UP;
    float feed = 1.0f;
    if (argc > 4) {
      if      (ieq(argv[4], "up")) up = true;
      else if (ieq(argv[4], "dn")) up = false;
      else feed = atof(argv[4]);
    }
    float q[MAX_AXES];
    const IkStatus st = ikSolve(p, up, q, true);
    if (st != IK_OK) {
      // Re-solve without the limit check so the user can see how far off it is.
      float raw[MAX_AXES];
      if (ikSolve(p, up, raw, false) == IK_OK)
        o.printf("# unclamped solution would be %.3f %.3f %.3f\n", raw[0], raw[1], raw[2]);
      o.printf("err %s\n", ikStatusStr(st));
      return;
    }
    o.printf("# xyz %.2f %.2f %.2f (elbow %s) -> q %.3f %.3f %.3f\n",
             p.x, p.y, p.z, up ? "up" : "dn", q[0], q[1], q[2]);
    if (ieq(c, "ik")) { o.println("ok"); return; }
    if (!motion.moveJointsDeg(q, feed, err, sizeof(err))) { o.printf("err %s\n", err); return; }
    o.println("ok");
    return;
  }
  if (ieq(c, "line")) {
    if (argc < 4) { o.println("err usage: line <x> <y> <z> [segments] [feed]"); return; }
    const Pose to = { (float)atof(argv[1]), (float)atof(argv[2]), (float)atof(argv[3]) };
    const int   segs = (argc > 4) ? atoi(argv[4]) : 40;
    const float feed = (argc > 5) ? atof(argv[5]) : 0.5f;
    testLine(o, to, segs, feed);
    return;
  }

  // ---- TMC2209 ----
  if (ieq(c, "tmc")) {
    if (argc > 1) {
      const int a = axisArg(argv[1]);
      if (a < 0) { o.println("err need a single axis"); return; }
      drivers.report(o, a);
    } else drivers.reportAll(o);
    o.println("ok");
    return;
  }
  // ----------------------------------------------------------- servos ----
  //  J4/J5/J6 + gripper. Open loop: these report what was COMMANDED. There is
  //  no feedback, so a stalled servo looks identical to one that arrived.
  if (ieq(c, "sv")) {
    if (argc == 1) {
      o.println("# id name  deg     us    limits        range trim  inv  pin");
      for (uint8_t i = 0; i < N_SERVOS; i++) {
        const ServoCfg &k = SV[i];
        o.printf("# %u  %-5s %7.1f %5u  %6.1f..%-6.1f %5.0f %5.1f %c  GPIO%u\n",
                 i, k.name, servos.pos(i), servos.lastUs(i),
                 k.minDeg, k.maxDeg, k.rangeDeg, k.trimDeg,
                 k.invert ? 'Y' : 'n', k.pin);
      }
      o.printf("# gripper %.0f%%   pulses %s\n", servos.gripPct(),
               servos.attached() ? "RUNNING" : "stopped (servos are limp)");
      o.println("ok");
      return;
    }
    const int id = servoIndex(argv[1]);
    if (id < 0) { o.println("err usage: sv <J4|J5|J6|grip> <deg>"); return; }
    if (argc < 3) { o.printf("# %s %.1f deg  %u us\n", SV[id].name, servos.pos(id),
                             servos.lastUs(id)); o.println("ok"); return; }
    const char *why = nullptr;
    if (!servos.setDeg((uint8_t)id, atof(argv[2]), &why)) {
      o.printf("err %s: %s (%.1f..%.1f)\n", SV[id].name, why ? why : "rejected",
               SV[id].minDeg, SV[id].maxDeg);
      return;
    }
    o.printf("# %s -> %.1f deg  (%u us)\n", SV[id].name, servos.pos(id), servos.lastUs(id));
    o.println("ok");
    return;
  }

  if (ieq(c, "grip")) {
    if (argc < 2) { o.printf("# grip %.0f%%\n", servos.gripPct()); o.println("ok"); return; }
    float pct;
    if      (ieq(argv[1], "open"))  pct = 100.0f;
    else if (ieq(argv[1], "close") || ieq(argv[1], "shut")) pct = 0.0f;
    else                            pct = atof(argv[1]);
    const char *why = nullptr;
    if (!servos.grip(pct, &why)) { o.printf("err %s\n", why ? why : "rejected"); return; }
    o.printf("# grip %.0f%%  (%.1f deg, %u us)\n", servos.gripPct(),
             servos.pos(SV_GRIP), servos.lastUs(SV_GRIP));
    o.println("ok");
    return;
  }

  if (ieq(c, "svpark"))   { servos.park(); o.println("# servos parked"); o.println("ok"); return; }
  if (ieq(c, "svattach")) { servos.attachAll(); o.println("# pulses running"); o.println("ok"); return; }
  if (ieq(c, "svoff") || ieq(c, "svdetach")) {
    servos.detachAll();
    o.println("# pulses stopped - servos are limp and will not hold position");
    o.println("ok");
    return;
  }

  if (ieq(c, "svset")) {
    if (argc < 4) {
      o.println("err usage: svset <J4|J5|J6|grip> <key> <v>");
      o.println("# keys: pin range minus maxus min max trim inv park");
      return;
    }
    const int id = servoIndex(argv[1]);
    if (id < 0) { o.println("err no such servo"); return; }
    ServoCfg &k = SV[id];
    const char *key = argv[2];
    const float v   = atof(argv[3]);
    bool reattach = false;
    if      (ieq(key, "pin"))   { const char *w = pinWarning((int)v);
                                  if (w) { o.printf("err GPIO%d: %s\n", (int)v, w); return; }
                                  k.pin = (uint8_t)v; reattach = true; }
    else if (ieq(key, "range")) { k.rangeDeg = v; }
    else if (ieq(key, "minus")) { k.minUs = (uint16_t)v; reattach = true; }
    else if (ieq(key, "maxus")) { k.maxUs = (uint16_t)v; reattach = true; }
    else if (ieq(key, "min"))   { k.minDeg = v; }
    else if (ieq(key, "max"))   { k.maxDeg = v; }
    else if (ieq(key, "trim"))  { k.trimDeg = v; }
    else if (ieq(key, "inv"))   { k.invert = v != 0; }
    else if (ieq(key, "park"))  { k.parkDeg = v; }
    else { o.println("err unknown key. try: svset <id> ?"); return; }
    if (reattach) { servos.detachAll(); servos.attachAll(); }
    servos.setDeg((uint8_t)id, servos.pos(id));
    o.printf("# %s %s = %g   (save to keep it)\n", k.name, key, v);
    o.println("ok");
    return;
  }

  // ------------------------------------------------------------- desk -----
  //  The thing that actually limits J1..J3, which keep their full +/-180.
  if (ieq(c, "desk")) {
    if (argc >= 2) {
      GEO.deskZ = atof(argv[1]);
      if (argc >= 3) GEO.deskMargin = atof(argv[2]);
    }
    float q[MAX_AXES]; for (uint8_t i = 0; i < MAX_AXES; i++) q[i] = motion.jointDeg(i);
    const float sv3[3] = { servos.pos(SV_J4), servos.pos(SV_J5), servos.pos(SV_J6) };
    ArmPoints P; fkChain(q, sv3, P);
    o.printf("# desk Z %.1f  margin %.1f  -> floor %.1f mm\n",
             GEO.deskZ, GEO.deskMargin, GEO.deskZ + GEO.deskMargin);
    o.printf("# J3 z %.1f | J4 z %.1f | J5 z %.1f | J6 z %.1f | tip z %.1f\n",
             P.j3.z, P.j4.z, P.j5.z, P.j6.z, P.tip.z);
    o.printf("# tip xyz  %.1f %.1f %.1f\n", P.tip.x, P.tip.y, P.tip.z);
    const float cl = deskClearance(P);
    if (cl >= 0) o.printf("# CLEAR by %.1f mm\n", cl);
    else         o.printf("# !! %.1f mm THROUGH the floor - lowest point is %.1f\n",
                          -cl, lowestZ(P));
    o.println("ok");
    return;
  }

  if (ieq(c, "uart")) { uartDiag(o, argc, argv); return; }

  if (ieq(c, "pinmap")) {
    o.println("# axis  STEP  DIR   EN    addr    <- setpin <ax> <step|dir|en> <gpio>");
    for (uint8_t i = 0; i < N_AXES; i++) {
      o.printf("# %-5s %-5u %-5u %-5u %u\n",
               AX[i].name, PINS[i].step, PINS[i].dir, PINS[i].en, PINS[i].addr);
    }
    if (pinAudit(o)) o.println("# no conflicts, no unsafe pins");
    o.printf("# UART bus is fixed at TX GPIO%d / RX GPIO%d\n", TMC_A_TX, TMC_A_RX);
    o.println("ok");
    return;
  }

  // Make the firmware match the wiring that actually exists, rather than
  // rewiring the arm to match a table someone wrote down. Takes effect after
  // save + reboot, because the step generator binds its pins once at startup.
  if (ieq(c, "setpin")) {
    if (argc < 4) { o.println("err usage: setpin <ax> <step|dir|en> <gpio>"); return; }
    const int a = axisArg(argv[1]);
    if (a < 0) { o.println("err need a single axis"); return; }
    const int g = atoi(argv[3]);
    const char *w = pinWarning(g);
    if (w) {
      o.printf("err GPIO%d is not usable here: %s\n", g, w);
      o.println("# pick another pin. nothing was changed.");
      return;
    }
    const bool isEn = ieq(argv[2], "en");
    const int clash = pinConflict(a, g, isEn);
    if (clash >= 0) {
      o.printf("err GPIO%d is already used by %s. Two axes cannot share a STEP\n",
               g, AX[clash].name);
      o.println("# or DIR line - that is what reboots the board. Run: pinmap");
      return;
    }
    if      (ieq(argv[2], "step")) PINS[a].step = (uint8_t)g;
    else if (ieq(argv[2], "dir"))  PINS[a].dir  = (uint8_t)g;
    else if (isEn)                 PINS[a].en   = (uint8_t)g;
    else { o.println("err field must be step, dir or en"); return; }
    o.printf("# %s %s -> GPIO%d\n", AX[a].name, argv[2], g);
    o.println("# not live yet. run: save   then: reboot");
    o.println("ok");
    return;
  }

  if (ieq(c, "reboot")) {
    o.println("# restarting");
    o.flush();
    delay(80);
    ESP.restart();
    return;
  }

  // Hold DIR at a fixed level so it can be measured with a meter at BOTH ends
  // of the wire. The TMC2209 DIR input has an internal pull-down, so a broken
  // wire reads low and the motor only ever turns one way - which is exactly
  // what a stuck direction looks like.
  if (ieq(c, "dir")) {
    if (argc < 3) { o.println("err usage: dir <ax> <0|1>"); return; }
    const int a = axisArg(argv[1]);
    if (a < 0) { o.println("err need a single axis"); return; }
    const int lvl = atoi(argv[2]) ? HIGH : LOW;
    gpioReadable(PINS[a].dir);
    digitalWrite(PINS[a].dir, lvl);
    delay(5);
    o.printf("# %s DIR = GPIO%u, driven %s, reads back %d\n",
             AX[a].name, PINS[a].dir, lvl ? "HIGH" : "LOW",
             digitalRead(PINS[a].dir));
    o.println("# Measure DC volts to GND in two places:");
    o.printf("#   1. the GPIO%u pin on the ESP32 header\n", PINS[a].dir);
    o.println("#   2. the DIR pin on the driver module");
    o.println("# Both must follow this command. Right at the MCU but wrong at");
    o.println("# the driver = the wire. Wrong at both = the MCU pin.");
    o.println("ok");
    return;
  }

  // Direct, slow STEP pulses - bypasses the step generator entirely so you can
  // hear each one and meter the line. Desyncs the position counter, so re-zero.
  if (ieq(c, "stp")) {
    if (argc < 3) { o.println("err usage: stp <ax> <pulses> [ms apart]"); return; }
    const int a = axisArg(argv[1]);
    if (a < 0) { o.println("err need a single axis"); return; }
    if (!motion.isEnabled()) { o.println("err drivers disabled, run: en 1"); return; }
    const int n  = atoi(argv[2]);
    const int ms = (argc > 3) ? atoi(argv[3]) : 200;
    gpioReadable(PINS[a].step);
    o.printf("# %u direct pulses on GPIO%u, %d ms apart. DIR is left as it is.\n",
             n, PINS[a].step, ms);
    for (int i = 0; i < n; i++) {
      if (s_->available()) { while (s_->available()) s_->read(); o.println("# aborted"); break; }
      digitalWrite(PINS[a].step, HIGH);
      delayMicroseconds(20);
      digitalWrite(PINS[a].step, LOW);
      delay(ms);
    }
    o.println("# If the motor did not twitch once per pulse, the STEP path is");
    o.println("# broken: the wire, or the driver, or the motor coils.");
    o.println("# The position counter is now WRONG for this axis - re-run pos/zero.");
    o.println("ok");
    return;
  }

  if (ieq(c, "probe")) {
    for (uint8_t i = 0; i < N_AXES; i++) {
      const bool up = drivers.probe(i);
      o.printf("# %s addr %u bus %u : %s\n", AX[i].name, PINS[i].addr, PINS[i].bus,
               up ? "ONLINE" : "no reply - check UART wiring, VIO, MS1/MS2 straps");
      if (up) drivers.apply(i);
    }
    o.println("ok");
    return;
  }
  if (ieq(c, "cur") || ieq(c, "ms") || ieq(c, "sc") || ieq(c, "sg") || ieq(c, "tpwm")) {
    if (argc < 3) { o.printf("err usage: %s <ax|all> <value>\n", c); return; }
    // These are all just shorthands for the matching `set` key.
    const char *key = ieq(c,"cur") ? "ma" : (ieq(c,"ms") ? "us" : c);
    char *fake[4] = { (char *)"set", argv[1], (char *)key, argv[2] };
    if (!cmdSet(4, fake)) return;
    if (ieq(c, "cur") && argc > 3) {
      char *fake2[4] = { (char *)"set", argv[1], (char *)"hold", argv[3] };
      cmdSet(4, fake2);
    }
    return;
  }
  if (ieq(c, "cool")) {
    if (argc < 4) { o.println("err usage: cool <ax|all> <semin> <semax>"); return; }
    char *f1[4] = { (char *)"set", argv[1], (char *)"semin", argv[2] };
    char *f2[4] = { (char *)"set", argv[1], (char *)"semax", argv[3] };
    if (!cmdSet(4, f1)) return;
    cmdSet(4, f2);
    return;
  }
  if (ieq(c, "sgr")) {
    const int sel = (argc > 1) ? axisArg(argv[1]) : -1;
    if (sel == -2) { o.printf("err bad axis %s\n", argv[1]); return; }
    o.println("# SG_RESULT stream, press any key to stop");
    while (true) {
      if (s_->available()) { while (s_->available()) s_->read(); break; }
      FOR_AXES(sel, i) drivers.reportLine(o, i);
      motion.poll();
      delay(100);
    }
    o.println("ok");
    return;
  }

  // ---- tests ----
  if (ieq(c, "t") || ieq(c, "test")) { cmdTest(argc, argv); return; }

  o.printf("err unknown command %s, try ?\n", c);
}

void Console::cmdTest(int argc, char **argv) {
  Stream &o = *s_;
  if (argc < 2) { o.println("err usage: t <ik|reach|speed|sweep|rep> ..."); return; }
  testRun(o, argc - 1, argv + 1);
}

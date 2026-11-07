#include "tests.h"
#include "drivers.h"
#include "motion.h"
#include <math.h>
#include <stdlib.h>
#include <esp_random.h>
#include <driver/gpio.h>

// digitalRead() on a pin set to plain GPIO_MODE_OUTPUT is not guaranteed to
// return the driven level - the input buffer is off. Switching to
// INPUT_OUTPUT keeps the output driver live and makes readback mean something.
void gpioReadable(uint8_t pin) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT);
}

static bool aborted(Stream &o) {
  if (!o.available()) return false;
  while (o.available()) o.read();
  o.println("# aborted");
  return true;
}

// An axis whose pins clashed at boot never got a step generator, so
// motion.stepper() hands back nullptr. Dereferencing that is a LoadProhibited
// panic - every routine that touches a stepper directly must ask first.
static bool axisReady(Stream &o, int ax) {
  if (motion.connected(ax)) return true;
  o.printf("err %s has no step generator attached.\n", AX[ax].name);
  o.println("# Its STEP pin clashed with another axis at startup. Run: pinmap");
  return false;
}

static float frand(float lo, float hi) {
  return lo + (hi - lo) * ((float)(esp_random() >> 8) / (float)(1u << 24));
}

// ===========================================================================
//  t ik [n]   -- pure maths, no motion. Round-trips random joint poses
//  through FK -> IK -> FK and reports how much precision the solver loses.
//  Run this after ANY change to geometry or angle conventions.
// ===========================================================================
static void testIk(Stream &o, int n) {
  if (n < 1) n = 500;
  float worstCart = 0.0f, sumCart = 0.0f, worstJoint = 0.0f;
  int fails = 0, branchFlips = 0;

  for (int k = 0; k < n; k++) {
    if (aborted(o)) return;
    float q[MAX_AXES] = {0};
    for (uint8_t i = 0; i < 3; i++) q[i] = frand(AX[i].minDeg, AX[i].maxDeg);

    Pose p;
    fkPose(q, p);

    float q2[MAX_AXES];
    const bool up = (q[2] <= 0.0f);            // branch that produced this pose
    const IkStatus st = ikSolve(p, up, q2, false);
    if (st != IK_OK) { fails++; continue; }

    Pose p2;
    fkPose(q2, p2);
    const float dc = sqrtf((p.x-p2.x)*(p.x-p2.x) + (p.y-p2.y)*(p.y-p2.y) +
                           (p.z-p2.z)*(p.z-p2.z));
    sumCart += dc;
    if (dc > worstCart) worstCart = dc;

    float dj = 0.0f;
    for (uint8_t i = 0; i < 3; i++) dj = fmaxf(dj, fabsf(q[i] - q2[i]));
    if (dj > 1.0f) branchFlips++; else if (dj > worstJoint) worstJoint = dj;
  }

  o.printf("# samples        %d\n", n);
  o.printf("# IK failures    %d\n", fails);
  o.printf("# branch flips   %d  (IK picked the other elbow solution)\n", branchFlips);
  o.printf("# cartesian err  mean %.6f mm   worst %.6f mm\n",
           n ? sumCart / (float)n : 0.0f, worstCart);
  o.printf("# joint err      worst %.6f deg (excluding branch flips)\n", worstJoint);
  o.println(worstCart < 0.01f && fails == 0
              ? "# verdict: solver is consistent with the FK model"
              : "# verdict: LOOK AT THIS - solver and FK disagree");
  o.println("ok");
}

// ===========================================================================
//  t reach [n]  -- how much of the bounding box can the arm actually get to,
//  once joint limits are applied. Tells you whether your soft limits are
//  strangling the workspace before you go chasing IK bugs.
// ===========================================================================
static void testReach(Stream &o, int n) {
  if (n < 1) n = 2000;
  const float R = reachMax() + GEO.baseOffset;
  int reachable = 0, unreachable = 0, limited = 0;

  for (int k = 0; k < n; k++) {
    if (aborted(o)) return;
    Pose p = { frand(-R, R), frand(-R, R),
               frand(GEO.baseHeight - R, GEO.baseHeight + R) };
    float q[MAX_AXES];
    switch (ikSolve(p, ELBOW_UP, q, true)) {
      case IK_OK:              reachable++;   break;
      case IK_OUT_OF_LIMITS:   limited++;     break;
      default:                 unreachable++; break;
    }
  }
  o.printf("# sampled %d points in a %.0f mm cube around the base\n", n, 2 * R);
  o.printf("# reachable            %5d  (%.1f%%)\n", reachable, 100.0f * reachable / n);
  o.printf("# blocked by limits    %5d  (%.1f%%)\n", limited,   100.0f * limited   / n);
  o.printf("# geometrically out    %5d  (%.1f%%)\n", unreachable, 100.0f * unreachable / n);
  o.println("ok");
}

// ===========================================================================
//  t speed <ax> [maxHz] [stepHz] [dwellms]
//  Walks the step rate up and reports, at every rung, what the driver thinks
//  is happening AND what rate the step generator actually delivered. That
//  second number is the one that catches "I asked for 200 kHz and got 60".
//  The axis free-runs, so let it spin or unbolt the link first.
// ===========================================================================
static void testSpeed(Stream &o, int ax, float maxHz, float stepHz, uint32_t dwell) {
  if (!axisReady(o, ax)) return;
  if (!motion.isEnabled()) { o.println("err drivers disabled, run: en 1"); return; }
  if (maxHz  <= 0) maxHz  = 40000.0f;
  if (stepHz <= 0) stepHz = 2000.0f;
  if (dwell  == 0) dwell  = 700;

  o.println("# WARNING: this free-runs the axis. Make sure it can spin without");
  o.println("# hitting anything. Any keystroke stops it.");
  o.printf("# %-9s %-10s %-8s %-6s %-5s %s\n",
           "askHz", "actualHz", "tstep", "sg", "cs", "flags");

  char err[96];
  float lastGood = 0.0f;

  for (float hz = stepHz; hz <= maxHz; hz += stepHz) {
    if (aborted(o)) break;
    if (!motion.runContinuous(ax, 1, hz, err, sizeof(err))) { o.printf("err %s\n", err); break; }

    // let it settle, then measure the delivered rate over a known window
    delay(dwell);
    const long  p0 = motion.stepper(ax)->getCurrentPosition();
    const uint32_t t0 = micros();
    delay(200);
    const long  p1 = motion.stepper(ax)->getCurrentPosition();
    const uint32_t dt = micros() - t0;
    const float actual = (float)(p1 - p0) * 1e6f / (float)dt;

    uint16_t sg = 0; uint8_t cs = 0; uint32_t ts = 0;
    bool otpw = false, ot = false;
    if (drivers.online[ax]) {
      TMC2209Stepper &d = *drivers.d(ax);
      sg = d.SG_RESULT(); cs = d.cs_actual(); ts = d.TSTEP();
      otpw = d.otpw(); ot = d.ot();
    }
    o.printf("  %-9.0f %-10.0f %-8lu %-6u %-5u %s%s%s\n",
             hz, actual, (unsigned long)ts, sg, cs,
             otpw ? "OVERTEMP-WARN " : "", ot ? "OVERTEMP-SHUTDOWN " : "",
             (actual < hz * 0.9f) ? "LOSING STEPS" : "");

    if (ot) { o.println("# driver thermal shutdown, stopping"); break; }
    if (actual < hz * 0.9f) {
      o.printf("# delivered rate fell off at %.0f Hz. Last clean rung: %.0f Hz\n",
               hz, lastGood);
      break;
    }
    lastGood = hz;
  }
  motion.stopAll(false);
  motion.waitIdle(o, 8000);
  o.printf("# highest clean step rate: %.0f Hz  =  %.1f deg/s  =  %.2f rev/s at the motor\n",
           lastGood, lastGood / motion.stepsPerDeg(ax),
           lastGood / (AX[ax].fullSteps * AX[ax].microsteps));
  o.println("ok");
}

// ===========================================================================
//  t sweep <ax> <cycles> <degA> <degB> [feed]
//  Endurance / thermal soak. Watches the driver flags every cycle, which is
//  how you find out that 1400 mA was too optimistic before you smell it.
// ===========================================================================
static void testSweep(Stream &o, int ax, int cycles, float a, float b, float feed) {
  if (!axisReady(o, ax)) return;
  if (!motion.isEnabled()) { o.println("err drivers disabled, run: en 1"); return; }
  char err[96];
  uint32_t worstCycle = 0, bestCycle = 0xFFFFFFFF;
  const uint32_t tStart = millis();

  for (int c = 0; c < cycles; c++) {
    for (int leg = 0; leg < 2; leg++) {
      const float tgt = leg ? a : b;
      const uint32_t t0 = millis();
      if (!motion.moveAxisSteps(ax, motion.degToSteps(ax, tgt), feed, err, sizeof(err))) {
        o.printf("err %s\n", err);
        return;
      }
      if (!motion.waitIdle(o)) return;
      const uint32_t dt = millis() - t0;
      if (dt > worstCycle) worstCycle = dt;
      if (dt < bestCycle)  bestCycle  = dt;
    }
    if (drivers.online[ax]) {
      TMC2209Stepper &d = *drivers.d(ax);
      o.printf("# cycle %-4d  cs %-3u  otpw %d ot %d  t120 %d t143 %d  openload %d/%d\n",
               c + 1, d.cs_actual(), (int)d.otpw(), (int)d.ot(),
               (int)d.t120(), (int)d.t143(), (int)d.ola(), (int)d.olb());
      if (d.ot()) { o.println("# thermal shutdown, aborting"); break; }
    } else {
      o.printf("# cycle %-4d  (driver offline, no telemetry)\n", c + 1);
    }
  }
  o.printf("# %d cycles in %.1f s   leg time min %lu ms / max %lu ms\n",
           cycles, (millis() - tStart) / 1000.0f,
           (unsigned long)bestCycle, (unsigned long)worstCycle);
  o.println("ok");
}

// ===========================================================================
//  t rep <ax> <cycles>
//  Homing repeatability: back off, creep in, record where the switch actually
//  tripped -- WITHOUT rezeroing. The spread is your real mechanical accuracy
//  floor. No amount of microstepping beats a sloppy switch or a loose pulley.
// ===========================================================================
static void testRep(Stream &o, int ax, int cycles) {
  if (!axisReady(o, ax)) return;
  if (!motion.isEnabled()) { o.println("err drivers disabled, run: en 1"); return; }
  if (PINS[ax].limit < 0) { o.printf("err %s has no limit switch wired\n", AX[ax].name); return; }
  if (cycles < 2) cycles = 10;

  if (!motion.home(ax, false, o)) return;

  long  first = 0, lo = 0, hi = 0;
  double sum = 0;
  int    n = 0;
  char   err[96];
  const float backoff = 15.0f;

  for (int c = 0; c < cycles; c++) {
    if (aborted(o)) break;

    // back off well clear of the switch
    const float away = AX[ax].homeDeg - AX[ax].homeDir * backoff;
    if (!motion.moveAxisSteps(ax, motion.degToSteps(ax, away), 0.6f, err, sizeof(err))) {
      o.printf("err %s\n", err);
      return;
    }
    if (!motion.waitIdle(o)) return;

    // creep back in and freeze the counter the instant the switch closes
    const float hz = fmaxf(1.0f, (AX[ax].homeVel / 4.0f) * motion.stepsPerDeg(ax));
    if (!motion.runContinuous(ax, AX[ax].homeDir, hz, err, sizeof(err))) {
      o.printf("err %s\n", err);
      return;
    }
    const uint32_t t0 = millis();
    while (!motion.limitHit(ax)) {
      if (aborted(o) || millis() - t0 > 30000) { motion.stopAll(true); o.println("err seek failed"); return; }
      delay(1);
    }
    const long trip = motion.stepper(ax)->getCurrentPosition();
    motion.stepper(ax)->forceStop();
    delay(120);

    if (n == 0) { first = trip; lo = hi = trip; }
    if (trip < lo) lo = trip;
    if (trip > hi) hi = trip;
    sum += (double)(trip - first);
    n++;
    o.printf("# trip %-3d  %8ld steps  %+8.4f deg from first\n",
             n, trip, motion.stepsToDeg(ax, trip - first));
  }

  if (n >= 2) {
    const long spread = hi - lo;
    o.printf("# %d trips   spread %ld steps = %.4f deg = %.4f mm at %.0f mm radius\n",
             n, spread, motion.stepsToDeg(ax, spread),
             radians(motion.stepsToDeg(ax, spread)) * (GEO.l1 + GEO.l2), GEO.l1 + GEO.l2);
    o.printf("# mean drift %+.4f deg\n", motion.stepsToDeg(ax, (long)(sum / n)));
  }
  o.println("ok");
}

// ===========================================================================
//  line <x> <y> <z> [segments] [feed]
//  Straight line in Cartesian space: chop the path up, run IK on every knot,
//  drive the joints through them. This is where you find out whether your
//  geometry numbers are right, because a bad L1/L2 turns a line into an arc.
// ===========================================================================
void testLine(Stream &o, const Pose &to, int segments, float feed) {
  if (!motion.isEnabled()) { o.println("err drivers disabled, run: en 1"); return; }
  if (segments < 1) segments = 1;
  if (segments > 400) segments = 400;

  float q0[MAX_AXES];
  motion.jointsDeg(q0);
  Pose from;
  fkPose(q0, from);

  // Dry-run the whole path first. Failing halfway through a move is worse
  // than refusing to start.
  for (int k = 1; k <= segments; k++) {
    const float u = (float)k / (float)segments;
    Pose p = { from.x + (to.x - from.x) * u,
               from.y + (to.y - from.y) * u,
               from.z + (to.z - from.z) * u };
    float q[MAX_AXES];
    const IkStatus st = ikSolve(p, ELBOW_UP, q, true);
    if (st != IK_OK) {
      o.printf("err segment %d/%d at %.2f %.2f %.2f: %s\n",
               k, segments, p.x, p.y, p.z, ikStatusStr(st));
      return;
    }
  }

  o.printf("# line %.1f %.1f %.1f -> %.1f %.1f %.1f in %d segments\n",
           from.x, from.y, from.z, to.x, to.y, to.z, segments);

  char err[96];
  const uint32_t t0 = millis();
  for (int k = 1; k <= segments; k++) {
    const float u = (float)k / (float)segments;
    Pose p = { from.x + (to.x - from.x) * u,
               from.y + (to.y - from.y) * u,
               from.z + (to.z - from.z) * u };
    float q[MAX_AXES];
    ikSolve(p, ELBOW_UP, q, true);
    if (!motion.moveJointsDeg(q, feed, err, sizeof(err))) { o.printf("err %s\n", err); return; }
    if (!motion.waitIdle(o)) return;
  }

  float qEnd[MAX_AXES];
  motion.jointsDeg(qEnd);
  Pose reached;
  fkPose(qEnd, reached);
  const float e = sqrtf((reached.x-to.x)*(reached.x-to.x) +
                        (reached.y-to.y)*(reached.y-to.y) +
                        (reached.z-to.z)*(reached.z-to.z));
  o.printf("# done in %.2f s, endpoint error %.4f mm (step quantisation only)\n",
           (millis() - t0) / 1000.0f, e);
  o.println("ok");
}

// ===========================================================================
//  t pins <ax>
//  Splits "the firmware is not driving the pin" from "the driver is ignoring
//  it". Reports the driver's own view of itself, then moves the axis each way
//  while watching the DIR line and the step counter.
// ===========================================================================
static void testPins(Stream &o, int ax) {
  if (!axisReady(o, ax)) return;
  o.printf("# --- %s : STEP GPIO%u   DIR GPIO%u   EN GPIO%u ---\n",
           AX[ax].name, PINS[ax].step, PINS[ax].dir, PINS[ax].en);
  o.printf("# EN pin reads %d   (LOW = drivers energised)\n", digitalRead(PINS[ax].en));

  if (!drivers.ensure(ax)) {
    o.println("# DRIVER OFFLINE. It is running on power-up defaults, which means");
    o.println("# 256 microsteps while the firmware assumes what cfg says - so every");
    o.println("# move comes out a fraction of its true size. Fix this before");
    o.println("# blaming anything mechanical. Try: probe");
  } else {
    TMC2209Stepper &d = *drivers.d(ax);
    o.printf("# driver ONLINE  version 0x%02X\n", d.version());
    const uint16_t chipUs = tmcGetMicrosteps(d);
    o.printf("# microstep chip says %u (MRES=%u), cfg says %u %s\n",
             chipUs, tmcMres(d), AX[ax].microsteps,
             (chipUs == AX[ax].microsteps) ? "(match)" : "<-- MISMATCH");
    o.printf("# current CS=%u (%u mA)  otpw=%d ot=%d  openload A=%d B=%d\n",
             d.cs_actual(), d.cs2rms(d.cs_actual()), (int)d.otpw(), (int)d.ot(),
             (int)d.ola(), (int)d.olb());
    o.printf("# chopper %s   shaft-invert %d\n",
             AX[ax].stealth ? "StealthChop" : "SpreadCycle", (int)AX[ax].invert);
  }

  if (!motion.isEnabled()) {
    o.println("# motors are disabled, nothing will turn. run: en 1");
    o.println("ok");
    return;
  }

  char err[96];
  int dirSeen[2] = {-1, -1};
  gpioReadable(PINS[ax].dir);
  for (int s = 0; s < 2; s++) {
    const float d = (s == 0) ? +5.0f : -5.0f;
    const long before = motion.stepper(ax)->getCurrentPosition();
    const float target = motion.jointDeg(ax) + d;
    if (!motion.moveAxisSteps(ax, motion.degToSteps(ax, target), 0.25f, err, sizeof(err))) {
      o.printf("# %+.0f deg REFUSED: %s\n", d, err);
      continue;
    }
    delay(25);
    dirSeen[s] = digitalRead(PINS[ax].dir);
    if (!motion.waitIdle(o, 15000)) return;
    const long after = motion.stepper(ax)->getCurrentPosition();
    o.printf("# %+.0f deg -> DIR pin %d, counter %ld -> %ld (%+ld steps)\n",
             d, dirSeen[s], before, after, after - before);
  }

  if (dirSeen[0] >= 0 && dirSeen[0] == dirSeen[1]) {
    o.println("# ** DIR DID NOT CHANGE between the two directions. **");
    o.printf("# GPIO%u is not being driven, is shorted, or the wire is off.\n",
             PINS[ax].dir);
  }
  o.println("# If the counter moved but the motor did not: pulses are being sent,");
  o.println("# so look at the driver - current, STEP wire, or motor coils.");
  o.println("ok");
}

// ------------------------------------------------------------ dispatcher ---
void testRun(Stream &o, int argc, char **argv) {
  const char *w = argv[0];

  if (ieq(w, "pins")) {
    if (argc < 2) { o.println("err usage: t pins <ax>"); return; }
    const int a = axisArg(argv[1]);
    if (a < 0) { o.println("err need a single axis"); return; }
    testPins(o, a);
    return;
  }

  if (ieq(w, "ik"))    { testIk(o, argc > 1 ? atoi(argv[1]) : 0); return; }
  if (ieq(w, "reach")) { testReach(o, argc > 1 ? atoi(argv[1]) : 0); return; }

  if (ieq(w, "speed")) {
    if (argc < 2) { o.println("err usage: t speed <ax> [maxHz] [stepHz] [dwellms]"); return; }
    const int a = axisArg(argv[1]);
    if (a < 0) { o.println("err need a single axis"); return; }
    testSpeed(o, a,
              argc > 2 ? atof(argv[2]) : 0,
              argc > 3 ? atof(argv[3]) : 0,
              argc > 4 ? (uint32_t)atol(argv[4]) : 0);
    return;
  }
  if (ieq(w, "sweep")) {
    if (argc < 5) { o.println("err usage: t sweep <ax> <cycles> <degA> <degB> [feed]"); return; }
    const int a = axisArg(argv[1]);
    if (a < 0) { o.println("err need a single axis"); return; }
    testSweep(o, a, atoi(argv[2]), atof(argv[3]), atof(argv[4]),
              argc > 5 ? atof(argv[5]) : 1.0f);
    return;
  }
  if (ieq(w, "rep")) {
    if (argc < 2) { o.println("err usage: t rep <ax> [cycles]"); return; }
    const int a = axisArg(argv[1]);
    if (a < 0) { o.println("err need a single axis"); return; }
    testRep(o, a, argc > 2 ? atoi(argv[2]) : 10);
    return;
  }
  o.printf("err unknown test %s, try: ik reach speed sweep rep\n", w);
}

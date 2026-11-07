#include "Protocol.h"
#include "Arm.h"
#include "TmcBus.h"
#include <strings.h>
#include <stdlib.h>
#include <math.h>
#include <driver/gpio.h>

static bool ieq(const char* a, const char* b) { return strcasecmp(a, b) == 0; }

// ===========================================================================
//  diag units <joint>
//
//  Every factor between "I typed 90" and "the shaft turned". The two worst
//  bugs on this project - a lost gear ratio and a driver silently left at 256
//  microsteps - were both invisible at the command line and both obvious here.
//  If a move comes out the wrong size, read this before touching hardware.
// ===========================================================================
static void diagUnits(Print& out, uint8_t i) {
  const JointConfig& c = JCFG[i];
  out.printf("# --- %s units ---\n", c.name);

  if (c.kind != KIND_STEPPER) {
    out.printf("< kind=servo pulse=%u..%uus over %.1f..%.1f deg\n",
               c.servoMinUs, c.servoMaxUs, c.servoMinDeg, c.servoMaxDeg);
    const float span = c.servoMaxDeg - c.servoMinDeg;
    out.printf("< us_per_deg=%.3f\n",
               fabsf(span) < 1e-6f ? 0.0f : (c.servoMaxUs - c.servoMinUs) / span);
    return;
  }

  StepperJoint* sj = (StepperJoint*)arm.joint(i);
  if (!sj) return;
  const float spd = sj->stepsPerDegree();

  out.printf("< motor_steps_per_rev=%.0f\n", c.motorStepsPerRev);
  out.printf("< microsteps_cfg=%u\n", c.microsteps);
  if (tmcBus.online(i)) {
    const uint16_t chipUs = tmcReadMicrosteps(*tmcBus.chip(i));
    out.printf("< microsteps_chip=%u %s\n", chipUs,
               chipUs == c.microsteps ? "(match)" : "<-- MISMATCH, moves will be wrong");
  } else {
    out.println("< microsteps_chip=? driver offline <-- it is on power-up "
                "defaults, not your settings");
  }
  out.printf("< gear_ratio=%.4f\n", c.gearRatio);
  out.printf("< steps_per_deg=%.4f   deg_per_step=%.5f\n", spd, 1.0f / spd);
  out.printf("< pulses_for_90deg=%ld\n", sj->degreesToSteps(90.0f));
  out.printf("< motor_revs_for_90deg=%.3f\n",
             (90.0f * spd) / (c.motorStepsPerRev * c.microsteps));
  out.printf("< max_step_rate=%.0f Hz at vel=%.1f deg/s\n",
             sj->maxStepRate(), c.maxVelDeg);
  out.printf("< motor_rpm_at_max=%.1f\n",
             (c.maxVelDeg * c.gearRatio) / 6.0f);
}

// ===========================================================================
//  diag joint <joint>
//
//  Splits "the firmware is not driving the pin" from "the driver is ignoring
//  it" from "the motor cannot move". Reports the driver's own view of itself,
//  then moves the joint each way while watching the shaft bit and the counter.
// ===========================================================================
static void diagJoint(Print& out, uint8_t i) {
  const JointConfig& c = JCFG[i];
  Joint* jt = arm.joint(i);

  out.printf("# --- %s ---\n", c.name);
  if (!jt || !jt->attached()) {
    out.printf("< attached=0  <-- no hardware behind this joint. run: pins\n");
    return;
  }
  out.printf("< attached=1 kind=%s\n", c.kind == KIND_STEPPER ? "stepper" : "servo");

  if (c.kind != KIND_STEPPER) {
    out.printf("< pwm_gpio=%u pulse_now=%uus\n",
               JPIN[i].step, ((ServoJoint*)jt)->pulseFor(jt->position()));
    out.println("# a servo has no feedback: this shows what is commanded, not "
                "where the horn is.");
    return;
  }

  out.printf("< step_gpio=%u dir=uart-shaft en_gpio=%u en_level=%d\n",
             JPIN[i].step, JPIN[i].enable, digitalRead(JPIN[i].enable));

  if (!tmcBus.ensure(i)) {
    out.println("< driver=offline  <-- running on power-up defaults, so every "
                "move comes out a fraction of its true size");
    return;
  }
  TMC2209Stepper& t = *tmcBus.chip(i);
  out.printf("< driver=online version=0x%02X current_now=%umA otpw=%d ot=%d "
             "openload_a=%d openload_b=%d\n",
             t.version(), t.cs2rms(t.cs_actual()), (int)t.otpw(), (int)t.ot(),
             (int)t.ola(), (int)t.olb());

  if (!arm.isEnabled()) {
    out.println("# motors are de-energised, so nothing will turn. run: enable 1");
    return;
  }

  StepperJoint* sj = (StepperJoint*)jt;
  int  shaftSeen[2]  = {-1, -1};
  long stepsMoved[2] = {0, 0};

  for (int pass = 0; pass < 2; pass++) {
    const float delta = (pass == 0) ? +5.0f : -5.0f;
    const long before = sj->raw()->getCurrentPosition();
    const ErrCode e = jt->moveTo(jt->position() + delta, 0.25f);
    if (e != E_OK) { out.printf("< move%+0.0f=refused reason=%s\n", delta, errName(e)); continue; }
    delay(25);
    // Direction is the GCONF.shaft bit now, not a pin, so read it back off the
    // chip. That is a stronger check than watching a GPIO ever was: it proves
    // the write reached the inside of the TMC2209.
    shaftSeen[pass] = tmcBus.readShaftBit(i);
    const uint32_t t0 = millis();
    while (jt->isMoving() && millis() - t0 < 8000) { arm.poll(); delay(1); }
    const long after = sj->raw()->getCurrentPosition();
    stepsMoved[pass] = after - before;
    out.printf("< move%+0.0f shaft=%d steps=%+ld\n",
               delta, shaftSeen[pass], stepsMoved[pass]);
  }

  // Two independent things have to be true, and the old version of this test
  // only checked one of them. It reported "UART direction control works" off
  // a correctly flipped shaft bit while the joint had emitted zero steps.
  const bool shaftOk = (shaftSeen[0] >= 0) && (shaftSeen[1] >= 0) &&
                       (shaftSeen[0] != shaftSeen[1]);
  const bool stepsOk = (stepsMoved[0] > 0) && (stepsMoved[1] < 0);

  if (shaftSeen[0] < 0 || shaftSeen[1] < 0) {
    out.println("# !! could not read GCONF back. UART is the only way this "
                "firmware sets direction, so every move is refused until that "
                "is fixed. run: probe");
  } else if (!shaftOk) {
    out.println("# !! the shaft bit did not change between the two directions. "
                "The UART write is not reaching the driver.");
  } else {
    out.printf("# shaft bit flipped %d -> %d as asked: the UART reached the driver.\n",
               shaftSeen[0], shaftSeen[1]);
  }

  if (!stepsOk) {
    out.printf("# !! STEP COUNTS ARE WRONG: %+ld then %+ld. Expected one "
               "positive and one negative.\n", stepsMoved[0], stepsMoved[1]);
    if (stepsMoved[0] == 0 || stepsMoved[1] == 0)
      out.println("# !! A zero means the step generator emitted nothing at all "
                  "in that direction - the joint never got a direction pin it "
                  "would accept. Check: pins");
  } else if (shaftOk) {
    out.println("# direction control is fully working: shaft bit flips AND the "
                "counter moves both ways.");
  }
  out.println("# counter moved but the motor did not? then pulses are leaving "
              "the ESP32 and the problem is the driver, the STEP wire, or the coils.");
  out.println("# motor moved less than commanded? run: diag units");
}

// ===========================================================================
//  diag skip <joint> [deg] [cycles] [feed]
//
//  Lost steps are invisible to an open-loop controller: the counter always
//  claims it arrived. So drive out and back N times and let the hardware
//  answer. The counter coming home is arithmetic, not evidence - the physical
//  mark is the evidence.
// ===========================================================================
static void diagSkip(Print& out, uint8_t i, float deg, int cycles, float feed) {
  Joint* jt = arm.joint(i);
  if (!jt || !jt->attached()) {
    out.printf("< attached=0  <-- no hardware behind this joint. run: pins\n");
    return;
  }
  if (!arm.isEnabled()) {
    out.println("# motors de-energised. run: enable 1");
    return;
  }

  const float start = jt->position();
  const float legs[2] = { start + deg, start };

  out.printf("# %s: %d cycles of %+.1f/%.1f deg at feed %.2f\n",
             JCFG[i].name, cycles, legs[0], legs[1], feed);
  out.println("# line up a mark on the joint with one on the frame FIRST.");

  for (int c = 0; c < cycles; c++) {
    for (int leg = 0; leg < 2; leg++) {
      const ErrCode e = jt->moveTo(legs[leg], feed);
      if (e != E_OK) { out.printf("< aborted reason=%s\n", errName(e)); return; }
      const uint32_t t0 = millis();
      while (jt->isMoving() && millis() - t0 < 30000) { arm.poll(); delay(1); }
    }
    if (i < STEPPER_COUNT && tmcBus.online(i)) {
      TMC2209Stepper& t = *tmcBus.chip(i);
      if (t.otpw() || t.ot()) {
        out.printf("< cycle=%d OVERTEMP otpw=%d ot=%d - stopping\n",
                   c + 1, (int)t.otpw(), (int)t.ot());
        return;
      }
    }
  }

  out.printf("< done cycles=%d counter_back_at=%.3f\n", cycles, jt->position());
  out.println("# The counter is back where it started, by definition.");
  out.println("# Look at the MARK. Any gap is lost steps; gap/cycles is the");
  out.println("# loss per move. If it drifted: raise current, lower acc, or");
  out.println("# lower feed - and re-run until the mark comes home.");
}

void Protocol::cmdDiag(int argc, char** argv) {
  if (argc < 2) { replyErr(E_SYNTAX, "diag <joint|units|skip> <joint>"); return; }

  if (ieq(argv[1], "skip")) {
    if (argc < 3) { replyErr(E_SYNTAX, "diag skip <joint> [deg] [cycles] [feed]"); return; }
    const int a = parseJoint(argv[2]);
    if (a < 0) { replyErr(E_BAD_JOINT, argv[2]); return; }
    diagSkip(*s, (uint8_t)a,
             argc > 3 ? atof(argv[3]) : 90.0f,
             argc > 4 ? atoi(argv[4]) : 10,
             argc > 5 ? atof(argv[5]) : 0.5f);
    replyOk();
    return;
  }

  if (ieq(argv[1], "units")) {
    if (argc < 3) { replyErr(E_SYNTAX, "diag units <joint>"); return; }
    const int a = parseJoint(argv[2]);
    if (a < 0) { replyErr(E_BAD_JOINT, argv[2]); return; }
    diagUnits(*s, (uint8_t)a);
    replyOk();
    return;
  }
  if (ieq(argv[1], "joint")) {
    if (argc < 3) { replyErr(E_SYNTAX, "diag joint <joint>"); return; }
    const int a = parseJoint(argv[2]);
    if (a < 0) { replyErr(E_BAD_JOINT, argv[2]); return; }
    diagJoint(*s, (uint8_t)a);
    replyOk();
    return;
  }
  replyErr(E_SYNTAX, "diag joint|units <joint>");
}

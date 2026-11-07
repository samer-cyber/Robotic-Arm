#include "TmcBus.h"

TmcBus tmcBus;

// GPIO16/17 are UART2's native pins on the classic ESP32. UART1 defaults to
// GPIO9/10, which are wired to the SPI flash on WROOM modules; remapping it
// works on most core versions and silently does not on others.
#define TMC_PORT Serial2

// --------------------------------------------------------------- microsteps
uint8_t tmcReadMres(TMC2209Stepper& t) {
  return (uint8_t)((t.CHOPCONF() >> 24) & 0x0F);
}

uint16_t tmcReadMicrosteps(TMC2209Stepper& t) {
  const uint8_t m = tmcReadMres(t);
  return (m <= 8) ? (uint16_t)(256u >> m) : 0;
}

void tmcSetMicrosteps(TMC2209Stepper& t, uint16_t microsteps) {
  uint8_t mres;
  switch (microsteps) {
    case 256: mres = 0; break;
    case 128: mres = 1; break;
    case  64: mres = 2; break;
    case  32: mres = 3; break;
    case  16: mres = 4; break;
    case   8: mres = 5; break;
    case   4: mres = 6; break;
    case   2: mres = 7; break;
    default:  mres = 8; break;      // 1 = full step
  }
  uint32_t chop = t.CHOPCONF();
  chop &= ~(0x0FUL << 24);
  chop |= ((uint32_t)mres << 24);
  t.CHOPCONF(chop);
}

// --------------------------------------------------------------------- bus
void TmcBus::begin(Print& log) {
  TMC_PORT.begin(TMC_BAUD, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  delay(200);                     // let VS and the drivers' rails settle

  for (uint8_t i = 0; i < STEPPER_COUNT; i++) {
    drv[i] = new TMC2209Stepper(&TMC_PORT, TMC_RSENSE, JPIN[i].tmcAddr);
    drv[i]->begin();
    probe(i);
    if (alive[i]) applyConfig(i);
    log.printf("# %s driver addr %u: %s\n", JCFG[i].name, JPIN[i].tmcAddr,
               alive[i] ? "online" : "NO REPLY");
  }
  if (!allOnline()) {
    log.println("# !! A driver did not answer. It is running on power-up "
                "defaults, not your settings, so moves will be small and weak.");
    log.println("# !! Check: VS present (UART needs motor supply), the R7 "
                "solder jumper, and the MS1/MS2 straps. Then: probe");
  }
}

bool TmcBus::probe(uint8_t i) {
  if (i >= STEPPER_COUNT || !drv[i]) return false;
  // Retry: one dropped frame at boot used to mark a healthy driver dead for
  // the whole session, and every later config write was then discarded.
  alive[i] = false;
  for (uint8_t attempt = 0; attempt < 3 && !alive[i]; attempt++) {
    if (attempt) delay(20);
    alive[i] = (drv[i]->test_connection() == 0) && (drv[i]->version() == 0x21);
  }
  // A driver we cannot talk to is a driver whose direction we cannot set,
  // so forget what we thought the shaft bit was.
  if (!alive[i]) dirKnown[i] = false;
  return alive[i];
}

bool TmcBus::ensure(uint8_t i) {
  if (i >= STEPPER_COUNT || !drv[i]) return false;
  if (alive[i]) return true;
  if (!probe(i)) return false;
  applyConfig(i);
  return true;
}

bool TmcBus::allOnline() const {
  for (uint8_t i = 0; i < STEPPER_COUNT; i++) if (!alive[i]) return false;
  return true;
}

void TmcBus::applyConfig(uint8_t i) {
  if (i >= STEPPER_COUNT || !drv[i]) return;
  const JointConfig& c = JCFG[i];
  TMC2209Stepper& t = *drv[i];

  t.pdn_disable(true);        // PDN is the UART pin, not standstill reduction
  t.mstep_reg_select(true);   // microsteps come from the register, not MS1/2
  t.I_scale_analog(false);    // ignore the VREF trimpot, use the internal ref
  t.toff(5);                  // enables the chopper; 0 switches the stage off
  t.blank_time(24);

  t.rms_current(c.runCurrent_mA, c.holdFactor);
  t.iholddelay(8);
  tmcSetMicrosteps(t, c.microsteps);
  t.intpol(c.interpolate);

  t.en_spreadCycle(!c.stealthChop);
  t.pwm_autoscale(true);
  t.pwm_autograd(true);

  // GCONF.shaft carries TWO things: the static `invert` config, which says
  // which way the joint is bolted together, and the live direction of
  // travel. Re-assert whichever way we are currently going - writing plain
  // c.invert here would silently reverse a joint mid-session every time
  // anything touched the config.
  const bool positive = dirKnown[i] ? dirPos[i] : true;
  t.shaft(c.invert ^ (!positive));
  TMC_PORT.flush();
  dirPos[i] = positive;
  dirKnown[i] = true;
}

// --------------------------------------------------------------- direction
int TmcBus::readShaftBit(uint8_t i) {
  if (i >= STEPPER_COUNT || !drv[i] || !alive[i]) return -1;
  const uint32_t g = drv[i]->GCONF();
  if (g == 0xFFFFFFFFUL) return -1;          // bus read failed
  return (int)((g >> 3) & 1u);
}

bool TmcBus::writeShaft(uint8_t i, bool shaftBit) {
  if (i >= STEPPER_COUNT || !drv[i] || !alive[i]) return false;
  TMC2209Stepper& t = *drv[i];

  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    t.shaft(shaftBit);
    // HardwareSerial.write() only queues bytes. Without this flush the
    // step pulses can start while the direction datagram is still sitting
    // in the TX FIFO, and the first few steps go the old way.
    TMC_PORT.flush();
    delayMicroseconds(200);                  // let the driver latch it
    dirWrites++;
#if TMC_VERIFY_SHAFT
    if (readShaftBit(i) == (shaftBit ? 1 : 0)) return true;
#else
    return true;
#endif
  }
  return false;
}

bool TmcBus::setDirection(uint8_t i, bool positive) {
  if (i >= STEPPER_COUNT || !alive[i]) return false;
  if (dirKnown[i] && dirPos[i] == positive) return true;   // already there

  if (!writeShaft(i, JCFG[i].invert ^ (!positive))) {
    dirKnown[i] = false;        // we no longer know which way it will turn
    return false;
  }
  dirPos[i] = positive;
  dirKnown[i] = true;
  return true;
}

void TmcBus::applyAll() {
  for (uint8_t i = 0; i < STEPPER_COUNT; i++) if (alive[i]) applyConfig(i);
}

// ------------------------------------------------------------------ reports
void TmcBus::report(Print& out, uint8_t i) {
  if (i >= STEPPER_COUNT) return;
  if (!drv[i] || !alive[i]) {
    out.printf("# %s driver offline (addr %u)\n", JCFG[i].name, JPIN[i].tmcAddr);
    return;
  }
  TMC2209Stepper& t = *drv[i];
  const uint32_t status = t.DRV_STATUS();
  const uint8_t  cs     = t.cs_actual();
  const uint16_t chipUs = tmcReadMicrosteps(t);

  out.printf("# %s addr=%u version=0x%02X ifcnt=%u gstat=0x%02X\n",
             JCFG[i].name, JPIN[i].tmcAddr, t.version(), t.IFCNT(), t.GSTAT());
  out.printf("#   current set=%umA now=%umA cs=%u hold=%.2f\n",
             JCFG[i].runCurrent_mA, t.cs2rms(cs), cs, JCFG[i].holdFactor);
  out.printf("#   microsteps chip=%u (mres=%u) cfg=%u %s\n",
             chipUs, tmcReadMres(t), JCFG[i].microsteps,
             chipUs == JCFG[i].microsteps ? "" : "<-- MISMATCH");
  out.printf("#   chopper=%s intpol=%d stealthflag=%d\n",
             JCFG[i].stealthChop ? "stealth" : "spread",
             (int)JCFG[i].interpolate, (int)((status >> 30) & 1));
  out.printf("#   drvstatus=0x%08lX standstill=%d otpw=%d ot=%d\n",
             (unsigned long)status, (int)t.stst(), (int)t.otpw(), (int)t.ot());
  out.printf("#   temp t120=%d t143=%d t150=%d t157=%d\n",
             (int)t.t120(), (int)t.t143(), (int)t.t150(), (int)t.t157());
  out.printf("#   short2gnd a=%d b=%d  short2vs a=%d b=%d  openload a=%d b=%d\n",
             (int)t.s2ga(), (int)t.s2gb(), (int)t.s2vsa(), (int)t.s2vsb(),
             (int)t.ola(), (int)t.olb());
}

void TmcBus::reportBrief(Print& out, uint8_t i) {
  if (i >= STEPPER_COUNT || !drv[i] || !alive[i]) return;
  TMC2209Stepper& t = *drv[i];
  out.printf("%s cs=%u sg=%u tstep=%lu otpw=%d ot=%d ",
             JCFG[i].name, t.cs_actual(), t.SG_RESULT(),
             (unsigned long)t.TSTEP(), (int)t.otpw(), (int)t.ot());
}

// ------------------------------------------------------------- raw bus tests
static uint8_t tmcCrc(const uint8_t* d, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t b = d[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = ((crc >> 7) ^ (b & 0x01)) ? (uint8_t)((crc << 1) ^ 0x07)
                                      : (uint8_t)(crc << 1);
      b >>= 1;
    }
  }
  return crc;
}

static int busShoot(const uint8_t* tx, uint8_t txLen,
                    uint8_t* rx, uint8_t rxMax, uint32_t windowMs) {
  while (TMC_PORT.available()) TMC_PORT.read();
  TMC_PORT.write(tx, txLen);
  TMC_PORT.flush();
  int n = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < windowMs && n < rxMax)
    if (TMC_PORT.available()) rx[n++] = (uint8_t)TMC_PORT.read();
  return n;
}

// TX and RX sit on the same physical node, so the ESP32 must hear its own
// transmission. If it does not, the fault is on the MCU side and the drivers
// are not involved - which splits the search space in half immediately.
void TmcBus::echoTest(Print& out) {
  static const uint8_t pattern[4] = {0x55, 0xAA, 0x0F, 0xF0};
  uint8_t got[16];
  const int n = busShoot(pattern, 4, got, sizeof(got), 60);

  out.print("# sent 55 AA 0F F0  got ");
  for (int i = 0; i < n; i++) out.printf("%02X ", got[i]);
  if (!n) out.print("(nothing)");
  out.println();

  if (n >= 4 && got[0] == 0x55 && got[3] == 0xF0) {
    out.println("# pass: TX pin, RX pin, resistor and wiring are all good.");
    out.println("# If probe still fails the fault is the driver: VS missing, "
                "R7 jumper unbridged, or a dead chip.");
  } else if (n == 0) {
    out.printf("# FAIL: no echo. GPIO%d and GPIO%d share one node, so the "
               "ESP32 must hear itself.\n", TMC_UART_TX, TMC_UART_RX);
    out.println("# The RX path is broken and the drivers are innocent.");
  } else {
    out.println("# partial: something came back but not what was sent - "
                "baud mismatch or a marginal joint.");
  }
}

void TmcBus::addressScan(Print& out) {
  out.println("# raw IOIN read to addresses 0..3; expect 4 echo bytes then "
              "an 8-byte reply starting 05 FF 06");
  for (uint8_t a = 0; a < 4; a++) {
    uint8_t frame[4] = {0x05, a, 0x06, 0};
    frame[3] = tmcCrc(frame, 3);
    uint8_t got[24];
    const int n = busShoot(frame, 4, got, sizeof(got), 60);
    out.printf("# addr %u got ", a);
    for (int i = 0; i < n; i++) out.printf("%02X ", got[i]);
    if (n == 0)      out.print(" <- dead bus, not even an echo");
    else if (n <= 4) out.print(" <- echo only, nothing answered");
    else if (n >= 12 && got[4] == 0x05 && got[5] == 0xFF)
      out.printf(" <- REPLY, version 0x%02X", got[7]);
    out.println();
  }
}

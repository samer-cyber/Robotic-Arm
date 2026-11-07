#include "drivers.h"
#include <strings.h>

Drivers drivers;

// ===========================================================================
//  Which UART peripheral drives which bus.
//
//  On the classic ESP32, GPIO16/17 are UART2's *native* pins. UART1 defaults to
//  GPIO9/10, which are wired to the SPI flash on WROOM modules; remapping UART1
//  elsewhere works on most Arduino-ESP32 versions and quietly does not on
//  others. So the main bus uses the port that needs no remapping at all.
// ===========================================================================
HardwareSerial* busPort(uint8_t bus) {
#if ARM_BOARD == BOARD_ESP32_WROOM
  return (bus == 1) ? &Serial1 : &Serial2;
#else
  return (bus == 1) ? &Serial2 : &Serial1;
#endif
}

// ---------------------------------------------------------- microsteps ----
// CHOPCONF bits 24..27 hold MRES: 0 = 256 usteps, 1 = 128, ... 8 = full step.
uint8_t tmcMres(TMC2209Stepper &t) {
  return (uint8_t)((t.CHOPCONF() >> 24) & 0x0F);
}

uint16_t tmcGetMicrosteps(TMC2209Stepper &t) {
  const uint8_t m = tmcMres(t);
  return (m <= 8) ? (uint16_t)(256u >> m) : 0;
}

void tmcSetMicrosteps(TMC2209Stepper &t, uint16_t us) {
  uint8_t mres;
  switch (us) {
    case 256: mres = 0; break;
    case 128: mres = 1; break;
    case  64: mres = 2; break;
    case  32: mres = 3; break;
    case  16: mres = 4; break;
    case   8: mres = 5; break;
    case   4: mres = 6; break;
    case   2: mres = 7; break;
    default:  mres = 8; break;             // 1 = full step
  }
  uint32_t chop = t.CHOPCONF();
  chop &= ~(0x0FUL << 24);
  chop |=  ((uint32_t)mres << 24);
  t.CHOPCONF(chop);
}

// TMC UART frame CRC: poly x^8 + x^2 + x + 1, LSB-first over each byte.
static uint8_t tmcCRC(const uint8_t *d, uint8_t len) {
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

// Drain, transmit, then collect whatever appears for `windowMs`.
static int shoot(HardwareSerial *bus, const uint8_t *tx, uint8_t txLen,
                 uint8_t *rx, uint8_t rxMax, uint32_t windowMs) {
  while (bus->available()) bus->read();
  bus->write(tx, txLen);
  bus->flush();
  int n = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < windowMs && n < rxMax)
    if (bus->available()) rx[n++] = (uint8_t)bus->read();
  return n;
}

static void dumpHex(Stream &o, const uint8_t *b, int n) {
  for (int i = 0; i < n; i++) o.printf("%02X ", b[i]);
  if (!n) o.print("(nothing)");
}

// ===========================================================================
//  uart diagnostics
// ===========================================================================
void uartDiag(Stream &o, int argc, char **argv) {
  HardwareSerial *bus = busPort(0);
  const char *what = (argc > 1) ? argv[1] : "loop";

  if (strcasecmp(what, "pins") == 0) {
    o.printf("# bus A baud %d, 8N1\n", TMC_BAUD);
    o.printf("#   GPIO%-3d = ESP32 TX  -> 1k -> driver RX pads\n", TMC_A_TX);
    o.printf("#   GPIO%-3d = ESP32 RX  <-       driver TX pads\n", TMC_A_RX);
#if ARM_BOARD == BOARD_ESP32_WROOM
    o.println("#   peripheral: UART2 (native pins, no remap)");
#else
    o.println("#   peripheral: UART1 (remapped)");
#endif
    o.println("ok");
    return;
  }

  // ---- echo test: the single most useful measurement here -----------------
  if (strcasecmp(what, "loop") == 0) {
    static const uint8_t pat[4] = {0x55, 0xAA, 0x0F, 0xF0};
    uint8_t got[16];
    const int n = shoot(bus, pat, 4, got, sizeof(got), 60);

    o.print("# sent  55 AA 0F F0\n# got   ");
    dumpHex(o, got, n);
    o.println();

    const bool echoed = (n >= 4 && got[0] == 0x55 && got[1] == 0xAA &&
                         got[2] == 0x0F && got[3] == 0xF0);
    if (echoed) {
      o.println("# PASS - the ESP32 hears its own transmission.");
      o.println("# TX pin, RX pin, the resistor and the wire are all fine.");
      o.println("# If probe still fails, the fault is on the driver side:");
      o.println("# VS missing, or the chip is dead. Run: uart scan");
    } else if (n == 0) {
      o.println("# FAIL - nothing came back, not even the echo.");
      o.printf("# On a single-wire bus GPIO%d and GPIO%d sit on the SAME node,\n",
               TMC_A_TX, TMC_A_RX);
      o.println("# so the ESP32 must hear itself. Silence means the RX path is");
      o.println("# broken, and the drivers are not involved at all.");
      o.printf("# Next: pull everything off and jumper GPIO%d straight to GPIO%d,\n",
               TMC_A_TX, TMC_A_RX);
      o.println("# then run this again. Still nothing = wrong pin or wrong UART.");
    } else {
      o.println("# PARTIAL - something returned, but not what was sent.");
      o.println("# Baud mismatch, or a marginal/noisy connection.");
    }
    o.println("ok");
    return;
  }

  // ---- raw read datagram to each address ---------------------------------
  if (strcasecmp(what, "scan") == 0) {
    o.println("# Sending a raw IOIN read to addresses 0..3.");
    o.println("# Expect: 4 echo bytes, then an 8-byte reply starting 05 FF 06.");
    for (uint8_t a = 0; a < 4; a++) {
      uint8_t dg[4] = {0x05, a, 0x06, 0};      // 0x06 = IOIN, read
      dg[3] = tmcCRC(dg, 3);
      uint8_t got[24];
      const int n = shoot(bus, dg, 4, got, sizeof(got), 60);

      o.printf("# addr %u  sent %02X %02X %02X %02X  got ", a, dg[0], dg[1], dg[2], dg[3]);
      dumpHex(o, got, n);
      if (n == 0)      o.print("  <- dead bus, not even an echo");
      else if (n <= 4) o.print("  <- echo only, nothing answered");
      else if (n >= 12 && got[4] == 0x05 && got[5] == 0xFF)
        o.printf("  <- REPLY, version 0x%02X", got[7]);
      o.println();
    }
    o.println("ok");
    return;
  }

  o.println("err usage: uart <loop|scan|pins>");
}

// ===========================================================================
void Drivers::begin() {
  busPort(0)->begin(TMC_BAUD, SERIAL_8N1, TMC_A_RX, TMC_A_TX);
  bool needBusB = false;
  for (uint8_t i = 0; i < N_AXES; i++) if (PINS[i].bus == 1) needBusB = true;
  if (needBusB) busPort(1)->begin(TMC_BAUD, SERIAL_8N1, TMC_B_RX, TMC_B_TX);

  for (uint8_t i = 0; i < N_AXES; i++) {
    tmc[i] = new TMC2209Stepper(busPort(PINS[i].bus), R_SENSE, PINS[i].addr);
    tmc[i]->begin();
    if (PINS[i].diag >= 0) pinMode(PINS[i].diag, INPUT);
    probe(i);
    if (online[i]) apply(i);
  }
}

bool Drivers::probe(uint8_t i) {
  if (i >= N_AXES || !tmc[i]) return false;
  // test_connection() returns 0 on success. version() should read 0x21.
  // Retry: a single dropped frame at boot used to mark a healthy driver dead
  // for the whole session, and every later config write was then discarded.
  online[i] = false;
  for (uint8_t attempt = 0; attempt < 3 && !online[i]; attempt++) {
    if (attempt) delay(20);
    online[i] = (tmc[i]->test_connection() == 0) && (tmc[i]->version() == 0x21);
  }
  return online[i];
}

// Anything about to write to a chip calls this first. A driver that dropped
// off the bus gets re-probed and fully reconfigured rather than silently
// ignored - losing settings without saying so was the worst bug in here.
bool Drivers::ensure(uint8_t i) {
  if (i >= N_AXES || !tmc[i]) return false;
  if (online[i]) return true;
  if (!probe(i)) return false;
  apply(i);
  return true;
}

bool Drivers::allOnline() {
  for (uint8_t i = 0; i < N_AXES; i++) if (!online[i]) return false;
  return true;
}

void Drivers::apply(uint8_t i) {
  if (i >= N_AXES || !tmc[i]) return;
  AxisCfg &a = AX[i];
  TMC2209Stepper &t = *tmc[i];

  t.pdn_disable(true);          // PDN pin is UART, not standstill-reduction
  t.mstep_reg_select(true);     // microsteps come from the register, not MS1/2
  t.I_scale_analog(false);      // ignore the VREF trimpot, use internal ref
  t.toff(5);                    // enables the chopper; 0 = driver off
  t.blank_time(24);

  t.rms_current(a.runmA, a.holdMult);
  t.iholddelay(8);
  tmcSetMicrosteps(t, a.microsteps);     // straight to CHOPCONF.MRES
  t.intpol(a.intpol);

  t.en_spreadCycle(!a.stealth); // true = SpreadCycle, false = StealthChop
  t.pwm_autoscale(true);
  t.pwm_autograd(true);
  t.TPWMTHRS(a.tpwmthrs);

  t.TCOOLTHRS(a.tcoolthrs);
  t.semin(a.semin);             // 0 disables CoolStep
  t.semax(a.semax);
  t.sedn(0b01);
  t.SGTHRS(a.sgthrs);

  t.shaft(a.invert);            // flip rotation inside the chip
}

void Drivers::applyAll() { for (uint8_t i = 0; i < N_AXES; i++) if (online[i]) apply(i); }

void Drivers::report(Stream &s, uint8_t i) {
  if (i >= N_AXES) { s.println("err bad axis"); return; }
  if (!tmc[i] || !online[i]) {
    s.printf("%s: OFFLINE (addr %u, bus %u) - no UART reply\n",
             AX[i].name, PINS[i].addr, PINS[i].bus);
    return;
  }
  TMC2209Stepper &t = *tmc[i];
  const uint32_t st  = t.DRV_STATUS();
  const uint8_t  cs  = t.cs_actual();

  s.printf("--- %s  (addr %u, bus %u) ---\n", AX[i].name, PINS[i].addr, PINS[i].bus);
  s.printf("  version    0x%02X   IFCNT %u   GSTAT 0x%02X\n",
           t.version(), t.IFCNT(), t.GSTAT());
  s.printf("  current    run %u mA (set) / %u mA (CS=%u now)  hold x%.2f\n",
           AX[i].runmA, t.cs2rms(cs), cs, AX[i].holdMult);
  s.printf("  microstep  %u  (MRES=%u)   intpol %d\n",
           tmcGetMicrosteps(t), tmcMres(t), (int)AX[i].intpol);
  s.printf("  mode       %s   (DRV_STATUS.stealth=%d)   TPWMTHRS %lu\n",
           AX[i].stealth ? "StealthChop" : "SpreadCycle",
           (int)((st >> 30) & 1), (unsigned long)AX[i].tpwmthrs);
  s.printf("  coolstep   semin %u semax %u   TCOOLTHRS %lu\n",
           AX[i].semin, AX[i].semax, (unsigned long)AX[i].tcoolthrs);
  s.printf("  stallguard SGTHRS %u   SG_RESULT %u   TSTEP %lu\n",
           AX[i].sgthrs, t.SG_RESULT(), (unsigned long)t.TSTEP());
  s.printf("  DRV_STATUS 0x%08lX\n", (unsigned long)st);
  s.printf("  flags      standstill=%d  otpw=%d ot=%d  t120=%d t143=%d t150=%d t157=%d\n",
           (int)t.stst(), (int)t.otpw(), (int)t.ot(),
           (int)t.t120(), (int)t.t143(), (int)t.t150(), (int)t.t157());
  s.printf("             short2gnd A=%d B=%d  short2vs A=%d B=%d  openload A=%d B=%d\n",
           (int)t.s2ga(), (int)t.s2gb(), (int)t.s2vsa(), (int)t.s2vsb(),
           (int)t.ola(), (int)t.olb());
  if (PINS[i].diag >= 0)
    s.printf("  DIAG pin   %d = %d\n", PINS[i].diag, digitalRead(PINS[i].diag));
}

void Drivers::reportLine(Stream &s, uint8_t i) {
  if (i >= N_AXES || !tmc[i] || !online[i]) return;
  TMC2209Stepper &t = *tmc[i];
  s.printf("%s tstep=%-7lu sg=%-4u cs=%-3u otpw=%d ot=%d stst=%d\n",
           AX[i].name, (unsigned long)t.TSTEP(), t.SG_RESULT(), t.cs_actual(),
           (int)t.otpw(), (int)t.ot(), (int)t.stst());
}

void Drivers::reportAll(Stream &s) { for (uint8_t i = 0; i < N_AXES; i++) report(s, i); }

# Direction over UART, not over DIR pins

**Change:** GPIO 32, 22 and 27 are no longer driven. The physical DIR pins on
these TMC2209 modules are damaged, so rotation direction is now the driver's
`GCONF.shaft` bit, written over the existing half-duplex UART bus on
GPIO17 (TX) / GPIO16 (RX), addresses 0/1/2.

Firmware 1.2.0. Verified: `pio run` clean, zero warnings, 26.7% flash.

## What actually changed

| | before | after |
|---|---|---|
| direction | `FastAccelStepper::setDirectionPin(JPIN[i].dir)` | `TmcBus::setDirection(i, positive)` -> `GCONF.shaft` |
| DIR pins | 32 / 22 / 27 | `PIN_NONE`, and forced to `PIN_NONE` on config load |
| driver offline | wrong current + microstepping | **every move refused** with `driver-offline` |
| `setpin N dir` | accepted | rejected, `unsupported` |
| `diag joint N` | watched the DIR GPIO | reads `GCONF` back off the chip |

`shaft = invert XOR (moving negative)`. Two things share that bit — the static
`invert` config, which says which way the joint is bolted together, and the
live direction of travel. `TmcBus::applyConfig` re-asserts the current
direction rather than writing plain `invert`, or any config touch would
silently reverse a joint mid-session.

## Bug found on hardware, and the fix (1.2.0)

Version 1.1.0 did not work. Forward moves ran, reverse moves did nothing at
all — no motion, no error. `diag joint 1` showed exactly why:

```
< move+5 shaft=0 steps=+42
< move-5 shaft=1 steps=+0
```

The shaft bit flipped correctly, so the UART side was fine all along. The
problem was `FastAccelStepper`: **with no direction pin registered it silently
refuses to emit down-counting steps.** `FastAccelStepper.cpp` only writes a
queue entry when `count_up || (_dirPin != PIN_UNDEFINED)`, and `moveTo()`
returns `MOVE_OK` regardless — the ramp runs, the queue stays empty, the joint
sits still. 1.1.0 also discarded that return value, so there was nothing to
report even when the library did object.

The library has a mechanism for exactly this: drivers whose direction line is
not a GPIO.

- Direction pin is set to `PIN_EXTERNAL_FLAG | joint` (128, 129, 130). No GPIO
  is touched — `setDirectionPin()` routes external pins to a callback instead
  of `PIN_OUTPUT()`.
- `engine.setExternalCallForPin()` registers `tmcExternalDirPin()`, which
  writes the shaft bit over UART and returns the resulting pin state.
- FastAccelStepper **drains the step queue before asking for the change**, and
  if the callback reports the change has not landed it inserts a 2 ms pause and
  asks again. That is what makes a ~0.7 ms UART datagram safe here, and it also
  means mid-move reversals are handled by the library rather than by the
  hand-rolled stop-and-wait 1.1.0 used.
- The callback runs on FastAccelStepper's `StepperTask`, not in an ISR.

Three things changed alongside it:

- `moveTo()` / `moveToTimed()` now map `MoveResultCode` onto our error codes.
  A refused move can no longer look like a completed one.
- `Arm::poll()` escalates to E-STOP after 50 consecutive failed direction
  writes (~100 ms of 2 ms retries) — otherwise a bus that dies mid-move leaves
  the axis paused forever with nothing reported.
- `diag joint` now checks **step counts as well as the shaft bit**. The old
  test printed "UART direction control works" off a correct shaft bit while the
  joint had emitted zero steps, which is precisely the failure it should have
  caught.

## The hazard this was written around

FastAccelStepper reverses **inside** a move if you retarget across the current
position: it decelerates, stops, and comes back. With a real DIR pin that
reversal drives the pin. With no pin it is invisible to us — the step counter
would run backwards while the motor kept turning forwards, and the firmware
would report a position wrong by twice the overshoot, confidently.

As of 1.2.0 the library handles this itself: it drains the queue before calling
the direction callback, so the counter and the motor cannot disagree.
`prepareDirection()` survives for one reason only — when the joint is idle it
does the UART write in the caller's context, where the result can be verified
and the move refused if the bus is dead. From inside the callback the only way
to signal a problem is to stall the axis.

`Arm::moveAll()` sets **every** direction bit before issuing **any** move. Each
one is a UART round trip of a millisecond or two; doing them inside the move
loop would stagger the starts by that much per joint and break exactly the
coordination that function exists to provide.

## Cost and verification

- ~0.7 ms to send the datagram at 115200 baud, plus ~1.5 ms for the read-back.
  Only paid when the direction actually inverts — the state is cached per joint.
- `TMC_PORT.flush()` before stepping. `HardwareSerial::write()` only queues
  bytes; without the flush the first pulses can go out while the direction
  datagram is still in the TX FIFO.
- Every write is read back from `GCONF` and retried once. Set
  `TMC_VERIFY_SHAFT 0` in `ArmConfig.h` to skip the read-back if it proves
  unreliable on your bus — you lose the guarantee that a move goes the way you
  asked, which on an open-loop arm means driving into a hard stop.
- Probe failure clears the cached direction: a driver we cannot talk to is a
  driver whose direction we do not know.

## Acceptance test

```
probe          -> all three online (mandatory now)
enable 1
diag joint 0   -> "shaft flipped 0 -> 1 as asked. UART direction control works."
```

## Reconstructed code — please eyeball

While removing the old DIR-pin test from `Diagnostics.cpp` I over-deleted, and
lost the tail of `diagJoint()` plus the opening of `diagSkip()`. There is no
git history here, so I rebuilt them from the string literals in the previous
`firmware.elf`. The literals and the surviving body are exact; the few lines of
scaffolding between them are my reconstruction:

- `diagSkip()` signature, its two guard clauses, `start`, `legs[2]`, and the
  `for (int c = 0; c < cycles; c++)` header.
- The header printf now prints `legs[0], legs[1]`. It may originally have
  printed `deg, start` — identical output whenever the joint starts at 0, which
  is why the ELF cannot settle it.

Behaviour is unchanged as far as I can tell, and it compiles and links clean,
but it is the one part of this change I could not verify against the original.

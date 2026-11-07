#pragma once
#include "config.h"
#include "kinematics.h"

// Test routines. All of them are blocking and all of them abort on any
// keystroke, so you can never get stuck watching something misbehave.
void testRun(Stream &o, int argc, char **argv);
void testLine(Stream &o, const Pose &to, int segments, float feed);

// shared with console.cpp
bool ieq(const char *a, const char *b);
void gpioReadable(uint8_t pin);
int  axisArg(const char *t);

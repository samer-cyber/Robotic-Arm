#pragma once
#include "config.h"

// Line-oriented ASCII protocol. Every command answers with either a block of
// "# ..." comment lines followed by "ok", or a single "err <reason>".
// That is deliberate: it stays typeable by a human in a serial monitor while
// a host script can just read until it sees ok/err.
class Console {
public:
  void begin(Stream &s);
  void poll();
  Stream& io() { return *s_; }

private:
  Stream *s_ = nullptr;
  char    buf[192];
  size_t  len = 0;

  void dispatch(int argc, char **argv);
  void help();
  void status();
  void dumpCfg();
  bool cmdSet(int argc, char **argv);
  bool cmdGeo(int argc, char **argv);
  void cmdTest(int argc, char **argv);
  void monitor(uint32_t periodMs);
};

extern Console console;

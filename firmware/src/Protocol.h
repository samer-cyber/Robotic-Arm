#pragma once
#include "Joint.h"

// ===========================================================================
//  Line protocol, version 1.
//
//  Design rules, so the host side can be written once and trusted:
//
//   * One command per line. CR, LF or CRLF all end it. `#` starts a comment.
//   * EVERY command produces exactly one terminator: `ok` or `err <n> <name>`.
//     A host reads until it sees one; it never has to guess when a reply ends.
//   * Data lines start with `< ` and are key=value pairs. Human notes start
//     with `# ` and carry no machine meaning.
//   * Unsolicited events start with `! ` and may appear at any time, including
//     between a command and its terminator. A host must tolerate that.
//   * Nothing blocks. A move command returns immediately; poll `state` or wait
//     for the `! idle` event.
//
//  That last rule is why this is firmware and not a test script: a controller
//  that stops answering while an axis moves cannot be commanded to stop.
// ===========================================================================
class Protocol {
public:
  void begin(Stream& io);
  void poll();

  void emitEvent(const char* fmt, ...);

private:
  void handleLine(char* line);
  void dispatch(int argc, char** argv);

  void replyOk();
  void replyErr(ErrCode e, const char* detail = nullptr);

  // command handlers
  void cmdHelp();
  void cmdInfo();
  void cmdState();
  void cmdConfig(int argc, char** argv);
  void cmdSet(int argc, char** argv);
  void cmdPins(int argc, char** argv);
  void cmdMove(int argc, char** argv, bool relative);
  void cmdMoveAll(int argc, char** argv);
  void cmdTeach(int argc, char** argv);
  void cmdDriver(int argc, char** argv);
  void cmdDiag(int argc, char** argv);
  void cmdGrip(int argc, char** argv);
  void gripReport();

  Stream* s = nullptr;
  char    buf[256];
  size_t  len = 0;
  bool    wasMoving = false;
};

extern Protocol proto;

// Shared token parser: "0".."5", "J1".."J6", or "all" (-1). -2 = garbage.
int parseJoint(const char* token);

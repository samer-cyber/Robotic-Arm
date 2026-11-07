"""Transport and protocol framing for arm-fw.

This is the only place that knows what the wire looks like. Everything above it
- the GUI, the CLI, and eventually a ROS 2 node - talks in Python objects.

Protocol recap (see AI/Firmware commands.md):
    < key=value ...     data
    # text              human note, no machine meaning
    ! text              event, can arrive at ANY time
    ok                  command succeeded
    err <n> <name> ...  command failed
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    raise SystemExit("pyserial is missing.  pip install pyserial")


@dataclass
class Reply:
    ok: bool
    rows: list[dict] = field(default_factory=list)   # every "< " line, parsed
    notes: list[str] = field(default_factory=list)   # every "# " line
    raw: list[str] = field(default_factory=list)
    err_code: Optional[int] = None
    err_name: str = ""
    err_detail: str = ""

    @property
    def row(self) -> dict:
        """First data row, or an empty dict. Convenient for single-row replies."""
        return self.rows[0] if self.rows else {}

    def merged(self) -> dict:
        """All data rows flattened into one dict. For replies like `info`."""
        out: dict = {}
        for r in self.rows:
            out.update(r)
        return out

    def __bool__(self) -> bool:
        return self.ok

    def __str__(self) -> str:
        if self.ok:
            return "ok"
        return f"err {self.err_code} {self.err_name} {self.err_detail}".strip()


def parse_data_line(text: str) -> dict:
    """`J1 pos=1.5 moving=0` -> {'_': ['J1'], 'pos': '1.5', 'moving': '0'}"""
    out: dict = {"_": []}
    for tok in text.split():
        if "=" in tok:
            k, _, v = tok.partition("=")
            out[k] = v
        else:
            out["_"].append(tok)
    return out


class ArmLink:
    """One serial connection to one controller.

    Thread-safe: `command()` serialises access, so a GUI polling `state` in the
    background cannot interleave with a jog issued from the UI thread.
    """

    def __init__(self, port: str, baud: int = 115200, connect_timeout: float = 3.0,
                 transport=None, settle: float = 2.0):
        self.lock = threading.RLock()
        self.events: list[str] = []
        self.on_event = None          # optional callback(str)

        # `transport` lets a simulated controller be injected, so the whole host
        # stack can be developed and tested with no hardware attached.
        if transport is not None:
            self.ser = transport
            self._banner = self._drain(0.2)
            return

        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.2
        # Do not assert DTR/RTS: on most ESP32 boards that yanks the reset line
        # and reboots the controller the instant a host connects.
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()

        # Deassert again AFTER open. Setting these before open states an
        # intent; some drivers still pulse the lines as the tty comes up, and
        # on an ESP32 that pair is wired to the auto-reset transistors. A
        # previous program closing the port resets the board too (HUPCL), so
        # we can arrive mid-boot through no fault of our own.
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass

        # Wait out a possible boot before asking anything. 0.3 s was not
        # enough: this firmware probes three TMC2209s over UART and reads NVS
        # before it answers. Ask too early and the reply is simply lost, which
        # showed up as `info` returning nothing and the host concluding the
        # arm had no joints at all.
        time.sleep(max(0.0, settle))
        self.ser.reset_input_buffer()

        # Drain whatever the boot printed, so it cannot be mistaken for the
        # answer to the first real command.
        self._banner = self._drain(max(0.3, connect_timeout * 0.2))

    # ------------------------------------------------------------------ io --
    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def _note_event(self, text: str) -> None:
        self.events.append(text)
        if self.on_event:
            try:
                self.on_event(text)
            except Exception:
                pass

    def _consume(self, text: str, reply) -> bool:
        """Handle one COMPLETE line. True if it terminated the reply."""
        reply.raw.append(text)
        if text.startswith("! "):
            self._note_event(text[2:])
        elif text.startswith("< "):
            reply.rows.append(parse_data_line(text[2:]))
        elif text.startswith("# "):
            reply.notes.append(text[2:])
        elif text == "ok":
            reply.ok = True
            return True
        elif text.startswith("err"):
            bits = text.split(maxsplit=3)
            reply.err_code = int(bits[1]) if len(bits) > 1 and bits[1].isdigit() else -1
            reply.err_name = bits[2] if len(bits) > 2 else "error"
            reply.err_detail = bits[3] if len(bits) > 3 else ""
            return True
        return False

    def _read_lines(self, deadline: float):
        """Yield COMPLETE lines, reassembling fragments across reads.

        pyserial's readline() returns whatever has arrived when its timeout
        expires - with or without a trailing newline. At high baud over a
        usbipd bridge the data lands in bursts that split mid-line, so one
        reply line comes back as two pieces. The old loop treated each piece as
        a line: the first half parsed as a mangled row, the second half matched
        no prefix and was silently dropped. That is how `state` lost J3..J6
        while J1 and J2 came through, and why `info` sometimes returned nothing
        at all.

        Fix: keep a byte buffer, split on the delimiter ourselves, and only
        ever hand a whole line to the parser.
        """
        # Injected transports (armlink.fake) implement readline() only, and
        # they hand back whole lines by construction - there is nothing to
        # reassemble, and calling read() on them fails. Take the simple path.
        if not hasattr(self.ser, "read"):
            while time.time() < deadline:
                raw = self.ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", "replace").strip()
                if text:
                    yield text
            return

        buf = bytearray()
        while time.time() < deadline:
            n = getattr(self.ser, "in_waiting", 0) or 1
            chunk = self.ser.read(n)
            if chunk:
                buf.extend(chunk)
            while b"\n" in buf:
                raw, _, rest = buf.partition(b"\n")
                buf = bytearray(rest)
                text = raw.decode("utf-8", "replace").strip()
                if text:
                    yield text

    def _drain(self, seconds: float) -> list[str]:
        """Read whatever is there for `seconds`, as complete lines."""
        return list(self._read_lines(time.time() + seconds))

    def command(self, line: str, timeout: float = 8.0) -> Reply:
        """Send one command, read until its terminator, return a Reply.

        Events arriving mid-reply are peeled off and recorded rather than
        confusing the parse - the firmware is explicitly allowed to interleave
        them, so the host has to cope.
        """
        if "\n" in line or "\r" in line:
            raise ValueError("command must be a single line")

        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write((line + "\n").encode())
            self.ser.flush()

            reply = Reply(ok=False)
            deadline = time.time() + timeout
            for text in self._read_lines(deadline):
                if self._consume(text, reply):
                    return reply

            reply.err_code, reply.err_name = -1, "host-timeout"
            reply.err_detail = (f"no terminator within {timeout}s "
                                f"({len(reply.rows)} rows read)")
            return reply

    def pump(self) -> list[str]:
        """Collect events that arrived while nothing was being commanded."""
        with self.lock:
            if not self.ser.in_waiting:
                return []
            for text in self._drain(0.05):
                if text.startswith("! "):
                    self._note_event(text[2:])
        out, self.events = self.events, []
        return out

    def take_events(self) -> list[str]:
        out, self.events = self.events, []
        return out


def find_port(preferred: Optional[str] = None) -> str:
    if preferred:
        return preferred
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("no serial ports found - is the controller plugged in?")
    for p in ports:
        blob = f"{p.description} {p.manufacturer or ''}".lower()
        if any(t in blob for t in ("ch34", "cp210", "silicon", "ftdi", "usb-serial")):
            return p.device
    return ports[0].device


def list_serial_ports() -> list[tuple[str, str]]:
    return [(p.device, p.description) for p in list_ports.comports()]

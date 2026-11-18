#!/usr/bin/env python3
"""Capture the sysid logger's stream to runs/<timestamp>_<label>/raw.log, with a live status line.

    uv run capture.py --label passive                 # port auto-detected (/dev/cu.usbmodem*, then usbserial)
    uv run capture.py --port /dev/cu.usbmodem5C372451831 --label fc-psteps --names n1c1=FC,n1c2=RC,n3c1=ML,n3c2=MR
    uv run capture.py --replay runs/<run>/raw.log --speed 0   # feed a recording through the same pipeline

The console is a WCH CH343 USB-UART bridge (VID 0x1A86 PID 0x55D3) at 921600 baud.
Its DTR/RTS lines drive the ESP32-P4's reset and boot pins, so capture clears both
on the pyserial object before opening. macOS still raises them for an instant when
the tty is opened, so an open may reset the board once: expect the ROM's boot
lines (at another baud, so they arrive as garbage bytes; kept in raw.log, counted)
and a fresh #meta/#cols. The port is therefore opened once and kept open: start
the next experiment's file with '/run <label>' instead of restarting capture. If
the cable is pulled the file stays open, capture retries every 0.5 s, and on
replug (likely another reset, t_us from zero) it carries on in the same file;
the parser treats the clock going backwards as a new segment.

Typing (Enter sends):
    hdr | rate <hz> | nodes | stop | mark <text> | dump <node> | clearfaults <node>
    arm <node> <ch> | pstep <node> <ch> <delta_counts>                                -> to the firmware
    /run <label>     close this run's file, start runs/<timestamp>_<label>/ (port stays open)
    /send <text>     send a line verbatim (the node guard still applies)
    /quit            (or Ctrl-D) send stop and exit
    anything else    an operator note: logged as a host event with a host timestamp
Motion commands (arm, pstep, step) naming node 2, the carriage sliders, are refused.
While a move is running (a pstep sent, or any _tgt live), ANY key sends 'stop'.
Ctrl-C, exit, SIGTERM and SIGHUP send 'stop' too.
'dump <node>' replies with #cfg lines; they are saved into the run's meta.json.
"""

from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import math
import os
import platform
import re
import select
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from sysid import attitude, wire  # noqa: E402
from sysid.runlog import parse_names  # noqa: E402

PORT_GLOBS = ("/dev/cu.usbmodem*", "/dev/cu.wchusbserial*", "/dev/cu.usbserial*", "/dev/ttyACM*", "/dev/ttyUSB*")
STEP_RE = re.compile(r"^\s*step\s+(\d+)\s+(\d+)\s+(-?[\d.]+)\s+(\d+)", re.I)
PSTEP_RE = re.compile(r"^\s*pstep\s+n?(\d+)\s+c?(\d+)\s+([+-]?\d+)", re.I)
PSTEP_TIMEOUT_S = 15.0  # a pstep counts as running at most this long without a live _tgt
END_RE = re.compile(r"\b(stop|abort|guard|step end|done|refused)\b", re.I)


# ---------------------------------------------------------------- transports

class SerialTransport:
    """The MIB's console. Never closed on purpose: every open resets the board."""

    def __init__(self, port: str, baud: int):
        self.port_arg, self.baud = port, baud
        self.ser = None
        self.path = None
        self.next_try = 0.0
        self.opens = 0
        self.last_error = ""

    def resolve(self) -> str | None:
        if self.port_arg and self.port_arg != "auto":
            return self.port_arg if os.path.exists(self.port_arg) else None
        for g in PORT_GLOBS:
            hits = sorted(glob.glob(g))
            if hits:
                return hits[0]
        return None

    def try_open(self) -> str | None:
        """Open if possible; returns a message on success."""
        if self.ser is not None or time.monotonic() < self.next_try:
            return None
        self.next_try = time.monotonic() + 0.5
        path = self.resolve()
        if not path:
            return None
        import serial

        try:
            s = serial.Serial()
            s.port, s.baudrate, s.timeout, s.write_timeout = path, self.baud, 0, 0.5
            # Clear DTR/RTS before open (as tools/mib_panel/mib_link.js does), so the CH343's
            # auto-reset circuit neither holds the P4 in reset nor selects the bootloader.
            # macOS may still pulse them during open(): the board can reset once.
            s.dtr = False
            s.rts = False
            s.open()
        except (OSError, ValueError, serial.SerialException) as e:
            self.last_error = f"cannot open {path}: {e}"
            return None
        self.last_error = ""
        self.ser, self.path = s, path
        self.opens += 1
        return path

    def fileno(self):
        try:
            return self.ser.fileno() if self.ser is not None else None
        except Exception:
            return None

    def read(self) -> bytes:
        if self.ser is None:
            return b""
        import serial

        try:
            return self.ser.read(65536)
        except (OSError, serial.SerialException):
            self.lost()
            raise ConnectionError("port lost")

    def write(self, data: bytes) -> bool:
        if self.ser is None:
            return False
        import serial

        try:
            # no flush(): it is tcdrain(), which waits forever on a wedged device; write_timeout bounds this
            self.ser.write(data)
            return True
        except (OSError, serial.SerialException):
            self.lost()
            return False

    def lost(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.next_try = time.monotonic() + 0.5

    @property
    def connected(self):
        return self.ser is not None

    def describe(self):
        return f"{self.path or self.port_arg} @ {self.baud}"


class ReplayTransport:
    """Feeds a recorded raw.log line by line, paced by its t_us (speed 0 = as fast as possible)."""

    def __init__(self, path: Path, speed: float):
        self.path = Path(path)
        self.lines = self.path.read_bytes().splitlines(keepends=True)
        self.i = 0
        self.speed = speed
        self.t_first = None
        self.h_first = None
        self.cols_t = 0
        self.connected = True
        self.opens = 1

    def try_open(self):
        return None

    def fileno(self):
        return None

    @property
    def eof(self):
        return self.i >= len(self.lines)

    def _t(self, line: bytes):
        s = line.decode("latin-1")
        if s.startswith("#cols:"):
            cols = wire.parse_cols(s)
            self.cols_t = cols.index("t_us") if "t_us" in cols else 0
            return None
        try:
            if s.startswith("D,"):
                return float(s[2:].split(",")[self.cols_t])
            if s.startswith("#evt,"):
                return float(s.split(",", 2)[1])
        except (ValueError, IndexError):
            return None
        return None

    def read(self) -> bytes:
        out = []
        now = time.monotonic()
        while self.i < len(self.lines) and len(out) < 400:
            line = self.lines[self.i]
            t = self._t(line)
            if t is not None and self.speed > 0:
                if self.t_first is None or t < self.t_first:
                    self.t_first, self.h_first = t, now
                due = self.h_first + (t - self.t_first) * 1e-6 / self.speed
                if due > now:
                    break
            out.append(line)
            self.i += 1
        return b"".join(out)

    def write(self, data: bytes) -> bool:
        return False

    def describe(self):
        return f"replay {self.path}"


# ---------------------------------------------------------------- the raw file

class RawWriter:
    """Device bytes go in untouched; host lines only at line boundaries."""

    def __init__(self, path: Path):
        self.path = path
        self.f = open(path, "ab")
        self.boundary = True
        self.pending: list[bytes] = []
        self.last_sync = time.monotonic()
        self.bytes = 0

    def device(self, data: bytes):
        if not data:
            return
        if self.pending:
            k = data.rfind(b"\n")
            if k >= 0 or self.boundary:
                head, tail = (data[: k + 1], data[k + 1:]) if k >= 0 else (b"", data)
                self.f.write(head)
                self.f.write(b"".join(self.pending))
                self.pending.clear()
                data = tail
        self.f.write(data)
        self.bytes += len(data)
        if data:
            self.boundary = data.endswith(b"\n")
        self.f.flush()
        if time.monotonic() - self.last_sync > 1.0:
            os.fsync(self.f.fileno())
            self.last_sync = time.monotonic()

    def host(self, line: bytes):
        if self.boundary:
            self.f.write(line)
            self.f.flush()
        else:
            self.pending.append(line)

    def break_line(self):
        """The device stopped mid-line (unplug): end that fragment so host lines stay whole."""
        if not self.boundary:
            self.f.write(b"\n")
            self.boundary = True
        if self.pending:
            self.f.write(b"".join(self.pending))
            self.pending.clear()
        self.f.flush()

    def close(self):
        self.break_line()
        self.f.flush()
        os.fsync(self.f.fileno())
        self.f.close()


# ---------------------------------------------------------------- session

def git_describe() -> str:
    try:
        return subprocess.run(["git", "-C", str(HERE), "describe", "--always", "--dirty", "--tags"],
                              capture_output=True, text=True, timeout=3).stdout.strip() or "unknown"
    except Exception:
        return "unknown"


class Session:
    def __init__(self, args, transport):
        self.a = args
        self.tr = transport
        self.parser = wire.LiveParser()
        self.runs_dir = Path(args.runs_dir)
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        self.names = parse_names(args.names)
        self.mount = attitude.load_mount(self.runs_dir / "_", args.level_ref, None)
        self.git = git_describe()
        self.run_dir = None
        self.writer = None
        self.meta = {}
        self.linebuf = b""
        self.meta_lines: dict[str, bytes] = {}
        self.cols_line: bytes | None = None
        self.step_until = 0.0
        self.step_desc = ""
        self.last_t_us = None
        self.last_row_host = None
        self.dropped = 0
        self.resets = 0
        self.period = None
        self.ui = None
        self.hdr_due = None
        self.connected_once = False
        self.pstep_grace = 0.0
        self.cfg_dirty = 0.0

    # -- runs
    def new_run(self, label: str):
        if self.writer:
            self.close_run()
        stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "-", label).strip("-") or "run"
        d = self.runs_dir / f"{stamp}_{safe}"
        n = 1
        while d.exists():
            n += 1
            d = self.runs_dir / f"{stamp}_{safe}-{n}"
        d.mkdir(parents=True)
        self.run_dir = d
        self.writer = RawWriter(d / "raw.log")
        self.meta = {"label": label, "port": self.tr.describe(), "baud": getattr(self.tr, "baud", None),
                     "start_time": dt.datetime.now().isoformat(timespec="milliseconds"), "end_time": None,
                     "git_describe": self.git, "host": platform.node(), "axis_names": self.names,
                     "operator_notes": [], "commands": [], "mount": self.mount.to_json(),
                     "replay_of": str(self.a.replay) if self.a.replay else None, "tool": "capture.py"}
        self.write_meta()
        self.host("sys", f"run start label={label} port={self.tr.describe()} git={self.git}")
        if self.cols_line:
            # this file starts mid-session: carry the header over so it parses on its own
            self.host("sys", "header carried over from the previous file")
            for line in self.meta_lines.values():
                self.writer.host(line)
            self.writer.host(self.cols_line)
        self.say(f"recording to {d}")

    def close_run(self):
        if not self.writer:
            return
        self.host("sys", "run end")
        self.meta["end_time"] = dt.datetime.now().isoformat(timespec="milliseconds")
        self.meta["counts"] = dict(self.parser.counts, dropped_rows_est=self.dropped, clock_resets=self.resets,
                                   port_opens=self.tr.opens)
        self.meta["fw_meta"] = self.parser.meta
        self.write_meta()
        self.writer.close()
        self.writer = None

    def write_meta(self):
        if not self.run_dir:
            return
        tmp = self.run_dir / "meta.json.tmp"
        tmp.write_text(json.dumps(self.meta, indent=1))
        os.replace(tmp, self.run_dir / "meta.json")

    # -- host events
    def t_us_est(self):
        if self.last_t_us is None or self.last_row_host is None:
            return None
        return self.last_t_us + (time.monotonic() - self.last_row_host) * 1e6

    def host(self, kind: str, text: str):
        if self.writer:
            self.writer.host(wire.format_host(kind, text, self.t_us_est()))

    def note(self, text: str):
        rec = {"time": dt.datetime.now().isoformat(timespec="milliseconds"), "t_us_est": self.t_us_est(), "text": text}
        self.meta.setdefault("operator_notes", []).append(rec)
        self.host("note", text)
        self.write_meta()
        self.say(f"note: {text}")

    def send(self, line: str, why: str = "") -> bool:
        refusal = wire.motion_refusal(line)
        if refusal:
            self.host("sys", f"{refusal}: {line.strip()}")
            self.meta.setdefault("commands", []).append({"time": dt.datetime.now().isoformat(timespec="milliseconds"),
                                                         "line": line.strip(), "sent": False, "why": refusal})
            self.say(refusal.upper())
            return False
        ok = self.tr.write((line.strip() + "\n").encode())
        tag = "" if ok else (" (replay, not sent)" if self.a.replay else " (NOT SENT: port not open)")
        self.host("cmd", line.strip() + tag + (f" [{why}]" if why else ""))
        self.meta.setdefault("commands", []).append({"time": dt.datetime.now().isoformat(timespec="milliseconds"),
                                                     "line": line.strip(), "sent": ok, "why": why})
        m = STEP_RE.match(line)
        if m and ok:
            self.step_until = time.monotonic() + int(m.group(4)) / 1000 + 0.5
            self.step_desc = f"n{m.group(1)}c{m.group(2)} {m.group(3)}%"
        m = PSTEP_RE.match(line)
        if m and ok:
            self.step_until = time.monotonic() + PSTEP_TIMEOUT_S
            self.pstep_grace = time.monotonic() + 1.0
            self.step_desc = f"pstep n{m.group(1)}c{m.group(2)} {m.group(3)}"
        if line.strip().lower() == "stop":
            self.step_until = 0.0
        if not ok and not self.a.replay:
            self.say(f"not sent: {line.strip()} (port not open)")
        return ok

    def stop(self, why: str):
        self.send("stop", why)
        self.say(f"stop sent ({why})")

    def live_targets(self):
        p = self.parser
        if not p.cols or self.last_row_host is None or time.monotonic() - self.last_row_host > 0.5:
            return []
        return [c for c in p.channels() if math.isfinite(p.value(f"{c}_tgt"))]

    @property
    def step_running(self):
        now = time.monotonic()
        if self.live_targets():
            return True
        if now < self.step_until and self.step_desc.startswith("pstep") and now > self.pstep_grace \
                and self.parser.index.get(next(iter(self.parser.channels()), "") + "_tgt") is not None:
            # the firmware logs _tgt and none is live: the move has ended
            self.step_until = 0.0
            return False
        return now < self.step_until

    # -- device bytes
    def feed(self, data: bytes):
        if self.writer:
            self.writer.device(data)
        buf = self.linebuf + data
        lines = buf.split(b"\n")
        self.linebuf = lines.pop()
        if len(self.linebuf) > 8192:  # garbage with no newline: don't grow forever
            self.linebuf = b""
        for raw in lines:
            self.line(raw + b"\n")

    def line(self, raw: bytes):
        kind, payload = self.parser.feed(raw)
        s = raw.decode("latin-1").rstrip("\r\n")
        if kind == wire.ROW:
            i = self.parser.index.get("t_us")
            t = payload[i] if i is not None else math.nan
            if math.isfinite(t):
                if self.last_t_us is not None:
                    d = t - self.last_t_us
                    if self.period is None:
                        r = self.parser.meta.get("sample_rate_hz")
                        try:
                            self.period = 1e6 / float(r) if r else None
                        except ValueError:
                            self.period = None
                        if self.period is None and 0 < d < 1e6:
                            self.period = d
                    if d < -50000:
                        self.resets += 1
                        self.say("device clock went backwards: new segment (reset?)")
                    elif self.period and d > 1.5 * self.period:
                        self.dropped += int(round(d / self.period)) - 1
                self.last_t_us = t
                self.last_row_host = time.monotonic()
        elif kind == wire.COLS:
            self.cols_line = raw
            self.period = None
        elif kind == wire.META:
            k, _ = wire.parse_meta(s)
            self.meta_lines[k] = raw
        elif kind == wire.EVT and payload:
            t, txt = payload
            self.say(f"evt {txt}")
            m = re.search(r"\bstep\s+n?(\d+)[\s,]*c?(\d+)\s+(-?[\d.]+)\s+(\d+)", txt)
            if m:
                self.step_until = max(self.step_until, time.monotonic() + int(m.group(4)) / 1000 + 0.3)
                self.step_desc = f"n{m.group(1)}c{m.group(2)} {m.group(3)}%"
            elif END_RE.search(txt):
                self.step_until = 0.0
        elif kind == wire.CFG and payload:
            self.meta.setdefault("controller_config", {}).setdefault(payload["node"], []).append(payload)
            self.cfg_dirty = time.monotonic()
        elif kind in (wire.STRAY, wire.LOG):
            n = self.parser.counts[wire.STRAY] + self.parser.counts[wire.LOG]
            if n <= 20 or self.a.verbose:
                self.say(("log " if kind == wire.LOG else "stray ") + s[:150])
            elif n == 21:
                self.say("more stray/log lines are kept in raw.log, counted, not shown (--verbose shows them)")

    # -- status
    def status(self) -> str:
        now = time.monotonic()
        p = self.parser
        if not self.tr.connected:
            head = "PORT LOST, retrying"
        elif p.cols is None:
            head = "waiting for header"
        elif self.last_row_host is None or now - self.last_row_host > 1.0:
            head = f"NO DATA {0 if self.last_row_host is None else now - self.last_row_host:.0f}s"
        else:
            head = "LIVE"
        parts = [f"{head}  {p.row_rate(now):5.1f} rows/s  drop {self.dropped}  garb {p.counts['garbled']}  "
                 f"stray {p.counts[wire.STRAY]}"]
        if p.cols:
            st = []
            for n in wire.nodes_in(p.cols):
                ages = [p.value(f"{c}_age_us") for c in p.channels() if c.startswith(n + "c")]
                ages = [a for a in ages if math.isfinite(a)]
                st.append(f"{n} {max(ages) / 1000:.0f}ms" if ages else f"{n} --")
            parts.append(" ".join(st))
            g = [p.value(f"imu_gr{a}") for a in "xyz"]
            if all(math.isfinite(v) for v in g):
                import numpy as np

                pitch, roll = attitude.from_gravity(np.array([g]), self.mount)
                parts.append(f"pitch {pitch[0]:+.2f} roll {roll[0]:+.2f}")
            enc = []
            for c in p.channels():
                v = p.value(f"{c}_enc")
                enc.append(f"{self.names.get(c, c)} {'--' if not math.isfinite(v) else int(v)}")
            parts.append(" ".join(enc))
        if self.step_running:
            live = self.live_targets()
            parts.append(f"MOVING {' '.join(self.names.get(c, c) for c in live) or self.step_desc} (any key = stop)")
        return " | ".join(parts)

    def say(self, msg: str):
        if self.ui:
            self.ui.message(msg)
        else:
            print(msg, flush=True)


# ---------------------------------------------------------------- terminal

class TTYUI:
    """Bottom two lines: status, then the input prompt. Messages scroll above."""

    def __init__(self, sess: Session):
        import termios
        import tty

        self.s = sess
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)  # keys one at a time; Ctrl-C still raises KeyboardInterrupt
        self.buf = ""
        self.out = sys.stdout
        self.out.write("\n\n")
        self.last = 0.0

    def restore(self):
        import termios

        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        self.out.write("\n")
        self.out.flush()

    def _bottom(self):
        w = shutil.get_terminal_size((120, 20)).columns
        st = self.s.status()[: w - 1]
        prompt = ("> " + self.buf)[-(w - 1):]
        return st, prompt

    def message(self, msg):
        st, prompt = self._bottom()
        stamp = dt.datetime.now().strftime("%H:%M:%S")
        self.out.write(f"\x1b[1A\r\x1b[2K{stamp} {msg}\n\r\x1b[2K{st}\n\r\x1b[2K{prompt}")
        self.out.flush()

    def redraw(self, force=False):
        now = time.monotonic()
        if not force and now - self.last < 0.25:
            return
        self.last = now
        st, prompt = self._bottom()
        self.out.write(f"\x1b[1A\r\x1b[2K{st}\n\r\x1b[2K{prompt}")
        self.out.flush()

    def fileno(self):
        return self.fd

    def poll(self):
        """Returns a list of submitted lines, or 'EOF'."""
        data = os.read(self.fd, 256).decode("utf-8", "replace")
        if self.s.step_running:
            self.s.stop("operator key during step")
            self.buf = ""
            self.redraw(True)
            return []
        out = []
        i = 0
        while i < len(data):
            ch = data[i]
            if ch == "\x1b":  # skip an escape sequence (arrows etc.)
                i += 1
                while i < len(data) and not data[i].isalpha() and data[i] != "~":
                    i += 1
            elif ch in "\r\n":
                out.append(self.buf)
                self.buf = ""
            elif ch in "\x7f\x08":
                self.buf = self.buf[:-1]
            elif ch == "\x15":
                self.buf = ""
            elif ch == "\x04":
                return "EOF"
            elif ch.isprintable():
                self.buf += ch
            i += 1
        self.redraw(True)
        return out


class PipeUI:
    """stdin is not a terminal (scripts, tests): one line = one entry; status printed every 2 s."""

    def __init__(self, sess: Session):
        self.s = sess
        self.fd = sys.stdin.fileno()
        self.partial = b""
        self.last = 0.0
        self.closed = False

    def restore(self):
        pass

    def message(self, msg):
        print(f"{dt.datetime.now().strftime('%H:%M:%S')} {msg}", flush=True)

    def redraw(self, force=False):
        if time.monotonic() - self.last >= 2.0 or force:
            self.last = time.monotonic()
            print("status: " + self.s.status(), flush=True)

    def fileno(self):
        return None if self.closed else self.fd

    def poll(self):
        """Returns a list of complete lines (possibly empty), or 'EOF' on a closed stdin with nothing left."""
        data = os.read(self.fd, 4096)
        if not data:
            self.closed = True
            rest, self.partial = self.partial, b""
            return [rest.decode("utf-8", "replace")] if rest.strip() else []
        if self.s.step_running:
            self.s.stop("operator input during step")
        self.partial += data
        *lines, self.partial = self.partial.split(b"\n")
        return [l.decode("utf-8", "replace") for l in lines]


# ---------------------------------------------------------------- main

def handle_input(sess: Session, line: str) -> bool:
    """Returns False to quit."""
    line = line.strip()
    if not line:
        return True
    if line in ("/quit", "/q", "/exit"):
        return False
    if line.startswith("/run"):
        label = line[4:].strip() or "run"
        sess.new_run(label)
        if sess.tr.connected:
            sess.send(f"mark run {label}", "new run file")
            sess.send("hdr", "new run file")
        return True
    if line.startswith("/send "):
        sess.send(line[6:])
        return True
    if line.startswith("/help"):
        for l in __doc__.split("Typing")[1].splitlines():
            sess.say(l)
        return True
    if line.split()[0].lower() in wire.FW_COMMANDS:
        sess.send(line)
        return True
    sess.note(line)
    return True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="auto")
    ap.add_argument("--baud", type=int, default=921600, help="ignored by USB-Serial-JTAG; matters for a UART bridge")
    ap.add_argument("--label", default="run")
    ap.add_argument("--names", help="axis names, n1c1=FC,n1c2=RC,n3c1=ML,n3c2=MR (stored in meta.json)")
    ap.add_argument("--runs-dir", type=Path, default=HERE / "runs")
    ap.add_argument("--level-ref", help="level_ref.json for the live pitch/roll (default runs/level_ref.json)")
    ap.add_argument("--replay", type=Path, help="feed a recorded raw.log through the pipeline instead of a port")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed; 0 = as fast as possible")
    ap.add_argument("--duration", type=float, help="exit after this many seconds")
    ap.add_argument("--verbose", action="store_true", help="print every stray / log line")
    a = ap.parse_args(argv)

    tr = ReplayTransport(a.replay, a.speed) if a.replay else SerialTransport(a.port, a.baud)
    sess = Session(a, tr)
    ui = TTYUI(sess) if sys.stdin.isatty() else PipeUI(sess)
    sess.ui = ui

    def on_signal(signum, frame):
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGHUP, on_signal)

    sess.new_run(a.label if not a.replay else (a.label if a.label != "run" else f"replay-{Path(a.replay).parent.name}"))
    t_start = time.monotonic()
    was_connected = False
    try:
        while True:
            if a.duration and time.monotonic() - t_start >= a.duration:
                break
            if not tr.connected:
                err_before = getattr(tr, "last_error", "")
                path = tr.try_open()
                err = getattr(tr, "last_error", "")
                if err and err != err_before:
                    sess.say(err + " (busy in another program? wrong --port?)")
                if path:
                    first = tr.opens == 1
                    sess.last_t_us = sess.last_row_host = None
                    sess.period = None
                    sess.host("sys", f"port opened {path}" + ("" if first else " again: device reset, new clock segment"))
                    sess.say(f"port {path} open" + ("" if first else " again (the board reset: expect boot text)"))
                    sess.hdr_due = time.monotonic() + 1.5
                    was_connected = True
            if sess.hdr_due and time.monotonic() >= sess.hdr_due and tr.connected:
                sess.hdr_due = None
                if sess.parser.cols is None:
                    sess.send("hdr", "no header yet after open")
            fds = [f for f in (tr.fileno(), ui.fileno()) if f is not None]
            try:
                r, _, _ = select.select(fds, [], [], 0.05) if fds else ([], [], [])
            except (OSError, ValueError):
                r = []
            if not fds:
                time.sleep(0.01 if a.replay else 0.05)
            if ui.fileno() is not None and ui.fileno() in r:
                got = ui.poll()
                if got == "EOF" or not all(handle_input(sess, line) for line in got):
                    break
            try:
                data = tr.read()
            except ConnectionError:
                if sess.writer:
                    sess.writer.break_line()
                sess.host("sys", "port lost")
                sess.say("port lost: keeping the file, retrying every 0.5 s")
                was_connected = False
                data = b""
            if data:
                sess.feed(data)
            if sess.cfg_dirty and time.monotonic() - sess.cfg_dirty > 0.5:
                sess.cfg_dirty = 0.0
                n = sum(len(v) for v in sess.meta.get("controller_config", {}).values())
                sess.write_meta()
                sess.say(f"controller config saved to meta.json ({n} entries)")
            if a.replay and tr.eof and not data:
                sess.say("replay finished")
                break
            ui.redraw()
    except KeyboardInterrupt:
        sess.say("interrupted")
    finally:
        try:
            sess.stop("capture exiting")
        finally:
            sess.close_run()
            ui.restore()
            print(f"saved {sess.run_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

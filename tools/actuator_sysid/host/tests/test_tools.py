"""The command-line tools end to end: capture.py on a replay and on a pseudo-terminal
(with garbage, an unplug and a replug), analyze.py on a whole run, simulate.py."""

import json
import os
import pty
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
PY = sys.executable


def _device_bytes(path: Path) -> bytes:
    """raw.log minus the host lines capture inserted (and carried-over headers)."""
    return b"".join(l for l in path.read_bytes().splitlines(keepends=True) if not l.startswith(b"#host,"))


def test_capture_replay_keeps_bytes_and_logs_notes(make, tmp_path):
    src = make("pstep").dir / "raw.log"
    inp = "chair on blocks\npstep 2 1 100\ndump 1\n"
    p = subprocess.run([PY, str(HERE / "capture.py"), "--replay", str(src), "--speed", "0", "--runs-dir",
                        str(tmp_path), "--label", "rt"], input=inp, capture_output=True, text=True, timeout=120)
    assert p.returncode == 0, p.stderr
    (d,) = list(tmp_path.glob("*_rt"))
    assert _device_bytes(d / "raw.log") == src.read_bytes()
    meta = json.loads((d / "meta.json").read_text())
    assert meta["operator_notes"][0]["text"] == "chair on blocks"
    refused = [c for c in meta["commands"] if c["line"] == "pstep 2 1 100"]
    assert refused and not refused[0]["sent"] and "node 2" in refused[0]["why"]
    assert set(meta["controller_config"]) == {"1", "2", "3"}
    assert meta["git_describe"] and meta["end_time"]
    text = (d / "raw.log").read_text("latin-1")
    assert "#host," in text and ",note,chair on blocks" in text


class FakeMIB:
    """Writes a recorded stream into a pty master; can be unplugged (master closed) and replugged."""

    def __init__(self, link: Path, data: bytes):
        self.link, self.data = link, data
        self.received = b""
        self.sent = b""
        self.master = None
        self._open()

    def _open(self):
        m, s = pty.openpty()
        self.master = m
        self.slave = s
        if self.link.exists() or self.link.is_symlink():
            self.link.unlink()
        self.link.symlink_to(os.ttyname(s))

    def write(self, b: bytes, timeout=5.0):
        """Non-blocking with a deadline, so a capture that stopped reading fails the test instead of hanging it."""
        os.set_blocking(self.master, False)
        end = time.monotonic() + timeout
        view = memoryview(b)
        while view:
            try:
                n = os.write(self.master, view)
                self.sent += bytes(view[:n])
                view = view[n:]
            except BlockingIOError:
                self.drain()
                if time.monotonic() > end:
                    raise TimeoutError("capture is not reading the port")
                time.sleep(0.01)

    def drain(self):
        import select

        while self.master is not None:
            r, _, _ = select.select([self.master], [], [], 0)
            if not r:
                break
            try:
                self.received += os.read(self.master, 4096)
            except OSError:
                break

    def unplug(self):
        self.drain()
        os.close(self.master)
        os.close(self.slave)
        self.master = None
        self.link.unlink()

    def replug(self):
        self._open()


def test_capture_on_a_pty_survives_garbage_and_replug(make, tmp_path):
    lines = (make("passive").dir / "raw.log").read_bytes().splitlines(keepends=True)
    link = tmp_path / "port"
    dev = FakeMIB(link, b"")
    # --baud 115200: macOS sets 921600 with an ioctl a pty rejects (the CH343 driver accepts it)
    proc = subprocess.Popen([PY, str(HERE / "capture.py"), "--port", str(link), "--baud", "115200", "--runs-dir",
                             str(tmp_path / "runs"), "--label", "pty", "--duration", "6"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT)
    time.sleep(1.2)
    first, second = lines[: len(lines) // 3], lines[len(lines) // 3: len(lines) // 2]
    dev.write(b"\x00\xe0\xfc ROM boot text at 115200\r\n")  # the bridge's reset: garbage first
    for k in range(0, len(first), 50):
        dev.write(b"".join(first[k:k + 50]))
        time.sleep(0.01)
    dev.write(b"D,123,45")  # a row cut off by the unplug
    time.sleep(0.4)
    dev.unplug()
    time.sleep(1.0)
    dev.replug()
    time.sleep(1.0)
    for k in range(0, len(second), 50):
        dev.write(b"".join(second[k:k + 50]))
        time.sleep(0.01)
    proc.stdin.write(b"note after replug\n")
    proc.stdin.flush()
    out, _ = proc.communicate(timeout=60)
    dev.drain()
    out = out.decode()
    assert proc.returncode == 0, out
    assert "port lost" in out and "open again" in out
    (d,) = list((tmp_path / "runs").glob("*_pty"))
    raw = (d / "raw.log").read_bytes()
    dev_bytes = _device_bytes(d / "raw.log")
    # everything the device sent is in the file, in order; the only addition is the newline
    # that ends the fragment cut off by the unplug
    assert dev_bytes.replace(b"D,123,45\n", b"D,123,45") == dev.sent
    assert b",sys,port lost" in raw and b"again: device reset" in raw and b",note,note after replug" in raw
    # stop on exit reached the (replugged) device; hdr was asked for after each open
    assert b"stop\n" in dev.received
    meta = json.loads((d / "meta.json").read_text())
    assert meta["counts"]["port_opens"] == 2


def test_analyze_all_writes_every_png_and_json(make, tmp_path):
    src = make("pstep").dir
    p = subprocess.run([PY, str(HERE / "analyze.py"), "all", str(src), "--level-ref", "none"], capture_output=True,
                       text=True, timeout=300)
    assert p.returncode == 0, p.stderr[-2000:]
    for name in ("overview", "rest", "pstep", "plant", "latency", "kinematics", "smoothness", "step", "deadband"):
        assert (src / f"{name}.png").read_bytes()[:4] == b"\x89PNG", name  # legacy ones draw an empty state here
        json.loads((src / f"{name}.json").read_text())


def test_simulate_cli_writes_a_run(tmp_path):
    p = subprocess.run([PY, str(HERE / "simulate.py"), "passive", "--out", str(tmp_path)], capture_output=True,
                       text=True, timeout=120)
    assert p.returncode == 0, p.stderr
    (d,) = list(tmp_path.iterdir())
    assert (d / "raw.log").exists() and (d / "sim_truth.json").exists() and (d / "meta.json").exists()

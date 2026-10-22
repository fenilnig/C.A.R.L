"""Link from the UNO Q Linux side to the STM32 MCU through arduino-router (msgpack-RPC over a unix socket).

Standard library only (the board has no msgpack module). The MCU sketch in firmware/carl_mcu provides:
    ctl(left_pm, right_pm, steer_ddeg, pan_ddeg, tilt_ddeg) -> [tof_mm, batt_mv, flags]
        (all int: wheel duty in per-mille -1000..1000, servo angles in tenths of a degree)
    ping() -> firmware version string

All the unit conversion lives here so it is testable on a laptop; the MCU only clamps and drives pins.
"""
from __future__ import annotations

import socket
import struct
import time

import dolly_core as d

ROUTER_SOCK = "/var/run/arduino-router.sock"

# ---- calibration (bench-tune these, see docs/WIRING_AND_CALIBRATION.md) ----------------------------
V_FREE_MPS = 2.6        # wheel speed at 100 % duty: 500 RPM x 0.1 m wheel. Open loop, no encoders.
STEER_CENTER_DEG = 90.0  # servo angle when the wheels point straight (set the trim on the bench)
STEER_GAIN = 1.0         # servo degrees per degree of virtual Ackermann angle (depends on the horn/tie-rod lever)
STEER_SIGN = 1.0         # flip to -1 if +steer turns the car right
PAN_CENTER_DEG = 90.0
TILT_CENTER_DEG = 90.0   # servo angle with the a6700 level
PAN_SIGN = 1.0
TILT_SIGN = 1.0

# flag bits returned by the MCU
FLAG_WATCHDOG = 1        # no ctl for 300 ms, motors were braked
FLAG_BATT_CUTOFF = 2     # pack below the MCU's own cutoff
FLAG_TOF_FAULT = 4       # sensor missing or timing out
FLAG_TOF_STOP = 8        # MCU hard-stopped forward motion on a very close obstacle


# ---- minimal msgpack ------------------------------------------------------------------------------
def pack(o) -> bytes:
    if o is None:
        return b"\xc0"
    if o is True:
        return b"\xc3"
    if o is False:
        return b"\xc2"
    if isinstance(o, int):
        if 0 <= o < 128:
            return bytes([o])
        if -32 <= o < 0:
            return struct.pack("b", o)
        if 0 <= o < 1 << 8:
            return b"\xcc" + struct.pack(">B", o)
        if 0 <= o < 1 << 16:
            return b"\xcd" + struct.pack(">H", o)
        if 0 <= o < 1 << 32:
            return b"\xce" + struct.pack(">I", o)
        if 0 <= o < 1 << 64:
            return b"\xcf" + struct.pack(">Q", o)
        if -(1 << 7) <= o < 0:
            return b"\xd0" + struct.pack(">b", o)
        if -(1 << 15) <= o < 0:
            return b"\xd1" + struct.pack(">h", o)
        if -(1 << 31) <= o < 0:
            return b"\xd2" + struct.pack(">i", o)
        return b"\xd3" + struct.pack(">q", o)
    if isinstance(o, float):
        return b"\xcb" + struct.pack(">d", o)
    if isinstance(o, str):
        b = o.encode()
        if len(b) < 32:
            return bytes([0xA0 | len(b)]) + b
        if len(b) < 256:
            return b"\xd9" + bytes([len(b)]) + b
        return b"\xda" + struct.pack(">H", len(b)) + b
    if isinstance(o, (list, tuple)):
        n = len(o)
        head = bytes([0x90 | n]) if n < 16 else b"\xdc" + struct.pack(">H", n)
        return head + b"".join(pack(x) for x in o)
    raise TypeError(f"cannot pack {type(o)}")


def unpack_one(buf: bytes, pos: int = 0):
    """Return (object, new_pos). Raises IndexError if buf ends mid-object (caller reads more)."""
    t = buf[pos]
    pos += 1

    def take(n):
        nonlocal pos
        if pos + n > len(buf):
            raise IndexError("incomplete")
        s = buf[pos:pos + n]
        pos += n
        return s

    if t < 0x80:
        return t, pos
    if t >= 0xE0:
        return t - 256, pos
    if 0xA0 <= t <= 0xBF:
        return take(t & 0x1F).decode(), pos
    if 0x90 <= t <= 0x9F or t == 0xDC:
        n = (t & 0x0F) if t != 0xDC else struct.unpack(">H", take(2))[0]
        out = []
        for _ in range(n):
            x, pos = unpack_one(buf, pos)
            out.append(x)
        return out, pos
    if 0x80 <= t <= 0x8F or t == 0xDE:
        n = (t & 0x0F) if t != 0xDE else struct.unpack(">H", take(2))[0]
        out = {}
        for _ in range(n):
            k, pos = unpack_one(buf, pos)
            v, pos = unpack_one(buf, pos)
            out[k] = v
        return out, pos
    fixed = {0xC0: None, 0xC2: False, 0xC3: True}
    if t in fixed:
        return fixed[t], pos
    fmt = {0xCC: ">B", 0xCD: ">H", 0xCE: ">I", 0xCF: ">Q", 0xD0: ">b", 0xD1: ">h", 0xD2: ">i",
           0xD3: ">q", 0xCA: ">f", 0xCB: ">d"}
    if t in fmt:
        return struct.unpack(fmt[t], take(struct.calcsize(fmt[t])))[0], pos
    if t == 0xD9:
        return take(take(1)[0]).decode(), pos
    if t == 0xDA:
        return take(struct.unpack(">H", take(2))[0]).decode(), pos
    if t in (0xC4, 0xC5):
        n = take(1)[0] if t == 0xC4 else struct.unpack(">H", take(2))[0]
        return bytes(take(n)), pos
    raise ValueError(f"unsupported msgpack type 0x{t:02x}")


# ---- RPC client -----------------------------------------------------------------------------------
class RpcError(Exception):
    pass


class RpcClient:
    """Blocking msgpack-RPC client. Requests are [0, id, method, params]; replies [1, id, error, result]."""

    def __init__(self, path: str = ROUTER_SOCK, timeout: float = 0.1):
        self.path, self.timeout = path, timeout
        self._sock = None
        self._buf = b""
        self._id = 0

    def _connect(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect(self.path)
        self._sock, self._buf = s, b""

    def close(self):
        if self._sock:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def call(self, method: str, params=()):
        if self._sock is None:
            self._connect()
        self._id = (self._id + 1) & 0x7FFFFFFF
        mid = self._id
        try:
            self._sock.sendall(pack([0, mid, method, list(params)]))
            deadline = time.monotonic() + self.timeout
            while True:
                try:
                    msg, n = unpack_one(self._buf)
                    self._buf = self._buf[n:]
                except IndexError:
                    if time.monotonic() > deadline:
                        raise TimeoutError("no reply from router")
                    chunk = self._sock.recv(4096)
                    if not chunk:
                        raise ConnectionError("router closed the socket")
                    self._buf += chunk
                    continue
                if isinstance(msg, list) and len(msg) == 4 and msg[0] == 1 and msg[1] == mid:
                    if msg[2] is not None:
                        raise RpcError(str(msg[2]))
                    return msg[3]
                # anything else (late replies, requests from the router) is ignored
        except (OSError, ValueError):
            self.close()
            raise


# ---- motor backend used by server.Dolly -----------------------------------------------------------
def duty(speed_mps: float) -> float:
    return max(-1.0, min(1.0, speed_mps / V_FREE_MPS))


def steer_servo_deg(steer_deg: float) -> float:
    return STEER_CENTER_DEG + STEER_SIGN * STEER_GAIN * steer_deg


def wire_params(left_duty, right_duty, steer_deg, pan_deg, tilt_deg):
    """Integers on the wire: duty in per-mille, angles in tenths of a degree (no float decoding on the MCU)."""
    return [round(left_duty * 1000), round(right_duty * 1000),
            round(steer_deg * 10), round(pan_deg * 10), round(tilt_deg * 10)]


class RouterMotors:
    """Drives the real car. Same interface as server.StubMotors.

    The MCU also has its own 300 ms watchdog, so a dead Linux side still stops the car.
    require_tof=True treats a missing/timing-out ToF sensor as an obstacle at 0 m (forward is blocked).
    Set require_tof=False (CARL_NO_TOF=1) only for bench tests with the sensor unplugged.
    """

    RETRY_S = 0.5

    def __init__(self, rpc=None, require_tof: bool = True, clock=time.monotonic):
        self.rpc = rpc or RpcClient()
        self.require_tof = require_tof
        self._clock = clock
        self.link = False
        self.flags = 0
        self._tof_mm = -1
        self._batt_mv = None
        self._next_try = 0.0

    def apply(self, speed, steer_deg, pan=0.0, tilt=0.0):
        now = self._clock()
        if not self.link and now < self._next_try:
            return
        left, right = d.rear_wheel_speeds(speed, steer_deg)
        params = wire_params(duty(left), duty(right), steer_servo_deg(steer_deg),
                             PAN_CENTER_DEG + PAN_SIGN * pan, TILT_CENTER_DEG + TILT_SIGN * tilt)
        try:
            res = self.rpc.call("ctl", params)
            self._tof_mm, self._batt_mv, self.flags = int(res[0]), int(res[1]), int(res[2])
            self.link = True
        except (OSError, RpcError, ValueError, IndexError, TypeError):
            self.link = False
            self._next_try = now + self.RETRY_S

    def read_tof_m(self):
        if self._tof_mm < 0 or self.flags & FLAG_TOF_FAULT:
            return 0.0 if self.require_tof else None
        return self._tof_mm / 1000.0

    def read_batt_v(self):
        return None if self._batt_mv is None else self._batt_mv / 1000.0

    def stop(self):
        try:
            self.rpc.call("ctl", wire_params(0.0, 0.0, STEER_CENTER_DEG, PAN_CENTER_DEG, TILT_CENTER_DEG))
        except (OSError, RpcError, ValueError, IndexError, TypeError):
            pass

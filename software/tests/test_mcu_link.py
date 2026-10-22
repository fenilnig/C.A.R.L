import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import dolly_core as d
import mcu_link as m


def test_msgpack_roundtrip():
    for o in (None, True, False, 0, 5, 127, 128, 255, 256, 65535, 65536, 2**32, -1, -32, -33, -129, -40000,
              1.5, -0.25, "ctl", "x" * 40, "y" * 300, [], [1, 2.5, "a", None, [True]], list(range(20))):
        got, n = m.unpack_one(m.pack(o))
        assert got == o and n == len(m.pack(o))


def test_unpack_incomplete_raises_indexerror():
    data = m.pack([0, 1, "ctl", [0.1, 0.2]])
    for cut in range(1, len(data)):
        try:
            m.unpack_one(data[:cut])
        except IndexError:
            continue
        raise AssertionError(f"cut at {cut} did not raise")


class FakeRpc:
    def __init__(self, reply=None, fail=False):
        self.calls, self.reply, self.fail = [], reply or [800, 11800, 0], fail

    def call(self, method, params=()):
        if self.fail:
            raise ConnectionError("router down")
        self.calls.append((method, list(params)))
        return self.reply


def test_apply_converts_units():
    rpc = FakeRpc()
    mo = m.RouterMotors(rpc)
    mo.apply(1.0, 0.0, pan=10.0, tilt=-5.0)
    method, p = rpc.calls[-1]
    assert method == "ctl"
    assert all(isinstance(x, int) for x in p) and len(p) == 5       # ints on the wire
    assert p[0] == round(1000 / m.V_FREE_MPS) and p[0] == p[1]      # straight: both wheels equal, per-mille
    assert p[2] == round(m.STEER_CENTER_DEG * 10)                    # tenths of a degree
    assert p[3] == round((m.PAN_CENTER_DEG + 10.0) * 10) and p[4] == round((m.TILT_CENTER_DEG - 5.0) * 10)
    mo.apply(1.0, 10.0)                                              # left turn: right wheel faster
    _, p = rpc.calls[-1]
    assert p[1] > p[0] and p[2] > round(m.STEER_CENTER_DEG * 10)


def test_duty_clamps():
    assert m.duty(99.0) == 1.0 and m.duty(-99.0) == -1.0 and m.duty(0.0) == 0.0


def test_telemetry_parsing():
    rpc = FakeRpc([800, 11800, 0])
    mo = m.RouterMotors(rpc)
    mo.apply(0.0, 0.0)
    assert mo.link and mo.read_tof_m() == 0.8 and mo.read_batt_v() == 11.8
    assert mo.read_batt_v() is not None and not d.is_low_battery(mo.read_batt_v())


def test_tof_fault_blocks_forward_unless_bench_mode():
    rpc = FakeRpc([-1, 11800, m.FLAG_TOF_FAULT])
    strict = m.RouterMotors(rpc)
    strict.apply(0.0, 0.0)
    assert strict.read_tof_m() == 0.0                    # sensor fault = obstacle at 0 m
    f = d.SafetyFilter(d.Deadman(10.0))
    f.deadman.beat()
    f.tof_m = strict.read_tof_m()
    assert f.process(1.0, 0.0).speed == 0.0
    bench = m.RouterMotors(rpc, require_tof=False)
    bench.apply(0.0, 0.0)
    assert bench.read_tof_m() is None


def test_link_loss_and_retry_throttle():
    t = [0.0]
    rpc = FakeRpc(fail=True)
    mo = m.RouterMotors(rpc, clock=lambda: t[0])
    mo.apply(1.0, 0.0)
    assert not mo.link
    rpc.fail = False
    mo.apply(1.0, 0.0)                                    # too soon, no retry yet
    assert not mo.link and rpc.calls == []
    t[0] = 0.6
    mo.apply(1.0, 0.0)
    assert mo.link and len(rpc.calls) == 1


def test_bad_reply_does_not_crash():
    mo = m.RouterMotors(FakeRpc(reply=["junk"]))
    mo.apply(0.0, 0.0)
    assert not mo.link


class _Sock:
    """Feeds a canned router reply in small chunks, like a real stream socket."""
    def __init__(self, reply):
        self.reply, self.sent = reply, b""

    def sendall(self, b): self.sent += b
    def recv(self, n):
        chunk, self.reply = self.reply[:3], self.reply[3:]
        return chunk
    def close(self): pass


def test_rpc_client_framing():
    c = m.RpcClient()
    c._sock = _Sock(m.pack([1, 1, None, [800, 11800, 0]]))
    assert c.call("ctl", [0.0]) == [800, 11800, 0]
    sent, _ = m.unpack_one(c._sock.sent)
    assert sent == [0, 1, "ctl", [0.0]]
    c._sock = _Sock(m.pack([1, 2, "boom", None]))
    try:
        c.call("ctl", [])
    except m.RpcError as e:
        assert "boom" in str(e)
    else:
        raise AssertionError("expected RpcError")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)

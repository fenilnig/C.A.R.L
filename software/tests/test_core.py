import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

import dolly_core as d


def test_ackermann_identity_equation():
    for delta in (3, 8, 15, 22):
        inner, outer = d.ackermann_angles(delta)
        lhs = 1 / math.tan(math.radians(outer)) - 1 / math.tan(math.radians(inner))
        assert abs(lhs - d.TRACK / d.WHEELBASE) < 1e-9
        assert inner > outer > 0


def test_ackermann_sign_and_zero():
    assert d.ackermann_angles(0) == (0.0, 0.0)
    i, o = d.ackermann_angles(-10)
    assert i < 0 and o < 0


def test_inner_limit_maps_to_20_deg():
    virtual = d.inner_limit_to_virtual_deg(20.0)
    inner, outer = d.ackermann_angles(virtual)
    assert abs(inner - 20.0) < 1e-6
    assert 15.0 < outer < 20.0       # outer must be smaller than inner
    assert virtual < 20.0


def test_rear_diff_straight_and_turn():
    assert d.rear_wheel_speeds(1.0, 0) == (1.0, 1.0)
    left, right = d.rear_wheel_speeds(1.0, 10)   # turning left
    assert left < 1.0 < right
    # mean of the two wheel speeds equals vehicle speed
    assert abs((left + right) / 2 - 1.0) < 1e-9
    # speed ratio matches radius ratio
    r = d.turn_radius(10)
    assert abs(left / right - (r - d.TRACK / 2) / (r + d.TRACK / 2)) < 1e-9
    l2, r2 = d.rear_wheel_speeds(1.0, -10)
    assert (l2, r2) == (right, left)


def test_speed_cap_by_lateral_accel():
    # tight turn: v^2/R <= 2.4
    virtual = d.inner_limit_to_virtual_deg()
    # at the 1 m/s demo cap the lateral limit does not bite (a_lat = v^2/R is about 1.2 m/s^2)
    assert d.limit_speed(1.0, virtual) == 1.0
    # with a higher speed ceiling the lateral limit does bite
    v = d.limit_speed(3.0, virtual, max_speed=3.0)
    r = d.turn_radius(virtual)
    assert v * v / r <= d.MAX_LATERAL_ACCEL + 1e-9
    assert v < 3.0
    # straight: only the demo cap applies
    assert d.limit_speed(5.0, 0) == d.MAX_SPEED
    assert d.limit_speed(-5.0, 0) == -d.MAX_SPEED


def test_stopping_distance_and_aeb():
    assert d.stopping_distance(0) == 0
    s = d.stopping_distance(1.0)
    assert abs(s - (0.15 + 1.0 / 6.0)) < 1e-9
    assert d.aeb_scale(1.0, 0.2) == 0.0
    assert d.aeb_scale(1.0, 5.0) == 1.0
    mid = d.aeb_scale(1.0, 1.25 * (s + 0.10))
    assert 0.0 < mid < 1.0
    # reversing is never blocked by the front sensor
    assert d.aeb_scale(-1.0, 0.05) == 1.0


def test_deadman_timing():
    t = [0.0]
    dm = d.Deadman(0.3, clock=lambda: t[0])
    assert not dm.alive()           # no heartbeat yet
    dm.beat()
    t[0] = 0.29
    assert dm.alive()
    t[0] = 0.31
    assert not dm.alive()


def test_safety_filter_blocks_when_unsafe():
    t = [0.0]
    f = d.SafetyFilter(d.Deadman(0.3, clock=lambda: t[0]))
    assert f.process(1, 0) == d.Command(0.0, 0.0)        # no heartbeat
    f.deadman.beat()
    cmd = f.process(1, 0)
    assert cmd.speed == d.MAX_SPEED
    f.estop = True
    assert f.process(1, 0) == d.Command(0.0, 0.0)
    f.estop = False
    f.low_battery = True
    assert f.process(1, 0) == d.Command(0.0, 0.0)
    f.low_battery = False
    f.tof_m = 0.2                                         # obstacle too close
    assert f.process(1, 0).speed == 0.0
    assert f.process(-1, 0).speed < 0.0                   # can still reverse


def test_safety_filter_clamps_inputs():
    f = d.SafetyFilter(d.Deadman(10.0))
    f.deadman.beat()
    cmd = f.process(9.0, -9.0)
    assert abs(cmd.speed) <= d.MAX_SPEED
    assert cmd.steer_deg < 0
    assert abs(cmd.steer_deg) <= d.inner_limit_to_virtual_deg() + 1e-9


def test_battery_divider():
    # full 3S (12.6 V) through 22k/6.8k at 3.3 V ref, 12-bit ADC
    v_pin = 12.6 * 6.8 / 28.8
    counts = round(v_pin / 3.3 * 4095)
    assert abs(d.battery_cells_voltage(counts, 12, 3.3) - 12.6) < 0.02
    assert d.is_low_battery(10.2)
    assert not d.is_low_battery(11.1)


def test_gimbal_integrates_and_holds():
    g = d.Gimbal()
    g.update(1.0, 0.0, 0.1)                       # full right for 0.1 s at 60 deg/s
    assert abs(g.pan - 6.0) < 1e-9 and g.tilt == 0.0
    g.update(0.0, 0.0, 0.1)                       # stick released: holds, does not drift back
    assert abs(g.pan - 6.0) < 1e-9
    g.recenter()
    assert (g.pan, g.tilt) == (0.0, 0.0)


def test_gimbal_limits_and_big_dt():
    g = d.Gimbal()
    for _ in range(200):
        g.update(5.0, 5.0, 0.1)                   # stick values are clamped to 1
    assert g.pan == d.PAN_LIMIT_DEG and g.tilt == d.TILT_MAX_DEG
    for _ in range(400):
        g.update(-1.0, -1.0, 0.1)
    assert g.pan == -d.PAN_LIMIT_DEG and g.tilt == d.TILT_MIN_DEG
    g.recenter()
    g.update(1.0, 0.0, 10.0)                      # a stalled loop must not jump the camera
    assert g.pan <= d.GIMBAL_RATE_DEG_S * 0.1 + 1e-9


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)

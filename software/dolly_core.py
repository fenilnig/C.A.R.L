"""Driving maths and safety layer for the Ground Dolly.

Pure Python, no hardware imports, so it runs on a laptop and on the UNO Q
Linux side. All lengths in metres, speeds in m/s, angles in degrees unless
the name ends in _rad.

Geometry (from the blueprint):
  track  w = 0.160 m   (front and rear)
  wheelbase L = 0.260 m
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

TRACK = 0.208          # Rev D, 40 mm wide wheels (was 0.200)
WHEELBASE = 0.260

# Practical limits
MAX_INNER_DEG = 20.0          # demo cap (mechanical limit is ~30 deg inner)
MAX_SPEED = 1.0               # m/s demo cap while carrying the a6700
MAX_LATERAL_ACCEL = 3.6       # m/s^2, half the est. Rev B rollover limit (7.2) with the a6700 aboard
DEADMAN_TIMEOUT_S = 0.30


def ackermann_angles(delta_virtual_deg: float, wheelbase: float = WHEELBASE, track: float = TRACK):
    """Return (inner_deg, outer_deg) for a virtual centre-wheel angle.

    Uses cot(outer) - cot(inner) = track / wheelbase. The sign of the input
    is the turn direction; the returned angles are signed the same way
    (inner is the wheel on the inside of the turn).
    """
    if abs(delta_virtual_deg) < 1e-9:
        return 0.0, 0.0
    sign = 1.0 if delta_virtual_deg > 0 else -1.0
    d = math.radians(abs(delta_virtual_deg))
    radius = wheelbase / math.tan(d)                       # to rear-axle midpoint
    inner = math.atan(wheelbase / (radius - track / 2.0))
    outer = math.atan(wheelbase / (radius + track / 2.0))
    return sign * math.degrees(inner), sign * math.degrees(outer)


def turn_radius(delta_virtual_deg: float, wheelbase: float = WHEELBASE) -> float:
    """Turning radius to the rear-axle midpoint. Infinite when driving straight."""
    if abs(delta_virtual_deg) < 1e-9:
        return math.inf
    return wheelbase / math.tan(math.radians(abs(delta_virtual_deg)))


def rear_wheel_speeds(speed: float, delta_virtual_deg: float, track: float = TRACK):
    """Electronic differential. Returns (left, right) rear wheel speeds in m/s.

    Positive steering angle = turn left, so the right wheel is the outer one.
    """
    r = turn_radius(delta_virtual_deg)
    if math.isinf(r):
        return speed, speed
    k = (track / 2.0) / r
    inner = speed * (1.0 - k)
    outer = speed * (1.0 + k)
    return (inner, outer) if delta_virtual_deg > 0 else (outer, inner)


def inner_limit_to_virtual_deg(max_inner_deg: float = MAX_INNER_DEG,
                               wheelbase: float = WHEELBASE, track: float = TRACK) -> float:
    """Largest virtual angle whose inner wheel angle equals max_inner_deg."""
    lo, hi = 0.0, 60.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if ackermann_angles(mid, wheelbase, track)[0] > max_inner_deg:
            hi = mid
        else:
            lo = mid
    return lo


def limit_speed(speed: float, delta_virtual_deg: float,
                max_speed: float = MAX_SPEED, max_lat: float = MAX_LATERAL_ACCEL) -> float:
    """Cap speed by the demo limit and by lateral acceleration v^2 / R <= max_lat."""
    sign = 1.0 if speed >= 0 else -1.0
    v = min(abs(speed), max_speed)
    r = turn_radius(delta_virtual_deg)
    if not math.isinf(r):
        v = min(v, math.sqrt(max_lat * r))
    return sign * v


def stopping_distance(speed: float, decel: float = 3.0, latency_s: float = 0.15) -> float:
    """Distance to stop from speed (m/s) with a reaction latency and constant decel."""
    v = abs(speed)
    return v * latency_s + v * v / (2.0 * decel)


def aeb_scale(speed: float, distance_m: float, decel: float = 3.0,
              latency_s: float = 0.15, margin_m: float = 0.10) -> float:
    """Return a 0..1 factor applied to forward speed from the ToF distance.

    1.0 = free, 0.0 = brake to zero. Linear ramp between the point where the
    stopping distance plus margin equals the distance, and 1.5x that.
    """
    if speed <= 0 or distance_m is None:
        return 1.0
    need = stopping_distance(speed, decel, latency_s) + margin_m
    if distance_m <= need:
        return 0.0
    if distance_m >= 1.5 * need:
        return 1.0
    return (distance_m - need) / (0.5 * need)


class Deadman:
    """Stops the car if no heartbeat arrives within the timeout."""

    def __init__(self, timeout_s: float = DEADMAN_TIMEOUT_S, clock=time.monotonic):
        self.timeout_s = timeout_s
        self._clock = clock
        self._last = None

    def beat(self):
        self._last = self._clock()

    def alive(self) -> bool:
        return self._last is not None and (self._clock() - self._last) <= self.timeout_s


@dataclass
class Command:
    speed: float      # m/s, signed
    steer_deg: float  # virtual angle, signed (+ left)


class SafetyFilter:
    """Turns raw joystick input into a safe motor command each control tick."""

    def __init__(self, deadman: Deadman | None = None):
        self.deadman = deadman or Deadman()
        self.estop = False
        self.low_battery = False
        self.tof_m: float | None = None

    def process(self, joy_throttle: float, joy_steer: float) -> Command:
        """joy_* are in -1..1. Returns a Command that is zero if unsafe."""
        if self.estop or self.low_battery or not self.deadman.alive():
            return Command(0.0, 0.0)
        max_virtual = inner_limit_to_virtual_deg()
        steer = max(-1.0, min(1.0, joy_steer)) * max_virtual
        speed = max(-1.0, min(1.0, joy_throttle)) * MAX_SPEED
        speed = limit_speed(speed, steer)
        if speed > 0 and self.tof_m is not None:
            speed *= aeb_scale(speed, self.tof_m)
        return Command(speed, steer)


PAN_LIMIT_DEG = 90.0           # +/- from centre, final gimbal has the a6700 on it
TILT_MIN_DEG = -30.0
TILT_MAX_DEG = 45.0
GIMBAL_RATE_DEG_S = 60.0       # full stick = this slew rate, keeps the a6700 shots smooth


class Gimbal:
    """Pan/tilt angle held by integrating a rate stick, so the camera never jumps.

    Stick values are -1..1 and mean angular rate. Angles clamp to the limits.
    No stick input leaves the angle where it is (it never drifts back to centre).
    """

    def __init__(self, pan_limit: float = PAN_LIMIT_DEG, tilt_min: float = TILT_MIN_DEG,
                 tilt_max: float = TILT_MAX_DEG, rate: float = GIMBAL_RATE_DEG_S):
        self.pan_limit, self.tilt_min, self.tilt_max, self.rate = pan_limit, tilt_min, tilt_max, rate
        self.pan = 0.0
        self.tilt = 0.0

    def update(self, pan_stick: float, tilt_stick: float, dt: float):
        dt = max(0.0, min(dt, 0.1))          # a stalled loop must not cause a big jump
        pan_stick = max(-1.0, min(1.0, pan_stick))
        tilt_stick = max(-1.0, min(1.0, tilt_stick))
        self.pan = max(-self.pan_limit, min(self.pan_limit, self.pan + pan_stick * self.rate * dt))
        self.tilt = max(self.tilt_min, min(self.tilt_max, self.tilt + tilt_stick * self.rate * dt))
        return self.pan, self.tilt

    def recenter(self):
        self.pan = 0.0
        self.tilt = 0.0


def battery_cells_voltage(adc_counts: int, adc_bits: int = 10, vref: float = 5.0,
                          r_top: float = 22_000.0, r_bottom: float = 6_800.0) -> float:
    """Pack voltage from the A0 divider (22k / 6.8k). Set vref=3.3 for UNO Q logic."""
    v_pin = adc_counts / ((1 << adc_bits) - 1) * vref
    return v_pin * (r_top + r_bottom) / r_bottom


def is_low_battery(pack_v: float, cells: int = 3, per_cell_v: float = 3.5) -> bool:
    return pack_v < cells * per_cell_v

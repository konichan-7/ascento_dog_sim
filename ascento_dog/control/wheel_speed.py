"""Wheel-speed drive control for the four-wheeled robot, independent of MuJoCo.

The module provides four independent pieces:

- ``DriveCommand``: a two-degree-of-freedom chassis-frame drive command.
- ``wheel_speed_targets``: maps a drive command to per-wheel angular speeds.
- ``WheelVelocityController``: a per-wheel PI speed controller with anti-windup.
- ``PulseTeleop``: latches W/A/S/D presses into linearly-decaying commands.

All quantities are SI (m, rad, s).  No physics engine is imported here.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from threading import Lock

import numpy as np


@dataclass(frozen=True)
class DriveCommand:
    """Two-degree-of-freedom drive command in the chassis frame.

    ``v_x`` is forward speed (m/s) along chassis +x; ``omega_yaw`` is the
    yaw rate (rad/s) about chassis +z, positive counterclockwise from above.
    """

    v_x: float
    omega_yaw: float


def wheel_speed_targets(
    command: DriveCommand,
    wheel_radius: float,
    mounts_y: Mapping[str, float],
) -> dict[str, float]:
    """Map a drive command to per-wheel target angular velocities (rad/s).

    Each wheel rolls only along the chassis +x axis.  The wheel-centre linear
    speed is ``v_x - omega_yaw * y`` (translation plus yaw lever arm).  In this
    model a wheel that spins positively about its +y axis rolls forward, so
    ``omega_wheel = (v_x - omega_yaw*y) / R``.
    """

    radius = float(wheel_radius)
    if not np.isfinite(radius) or radius <= 0.0:
        raise ValueError("wheel_radius must be finite and positive")
    if not np.isfinite(command.v_x) or not np.isfinite(command.omega_yaw):
        raise ValueError("drive command must be finite")
    if not mounts_y:
        raise ValueError("mounts_y must not be empty")
    targets: dict[str, float] = {}
    for name, y in mounts_y.items():
        y_value = float(y)
        if not np.isfinite(y_value):
            raise ValueError(f"mount y for {name!r} must be finite")
        targets[name] = (command.v_x - command.omega_yaw * y_value) / radius
    return targets


@dataclass(frozen=True)
class WheelSpeedGains:
    """Per-wheel speed PI gains and limits in SI units.

    ``kp`` has units N*m*s/rad, ``ki`` N*m/rad, ``integral_limit`` rad, and
    ``output_limit`` N*m.
    """

    kp: float
    ki: float
    integral_limit: float = np.inf
    output_limit: float = np.inf

    def __post_init__(self) -> None:
        for value in (self.kp, self.ki):
            if not np.isfinite(value) or value < 0.0:
                raise ValueError("wheel-speed PI gains must be finite and nonnegative")
        if np.isnan(self.integral_limit) or np.isnan(self.output_limit):
            raise ValueError("wheel-speed PI limits may not be NaN")
        if self.integral_limit <= 0.0 or self.output_limit <= 0.0:
            raise ValueError("wheel-speed PI limits must be positive")


class WheelVelocityController:
    """Per-wheel PI speed controller with conditional anti-windup.

    Each wheel integrates its own speed error; integration stops while the
    output is saturated and the error keeps pushing further out of range.
    """

    def __init__(self, gains: WheelSpeedGains) -> None:
        self.gains = gains
        self.integral: dict[str, float] = {}

    def reset(self) -> None:
        """Clear all integral state."""

        self.integral.clear()

    def update(
        self,
        targets: Mapping[str, float],
        measured: Mapping[str, float],
        dt: float,
    ) -> dict[str, float]:
        """Return per-wheel torque (N*m) to track ``targets`` (rad/s)."""

        if set(targets) != set(measured):
            raise ValueError("targets and measured must share the same wheel names")
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")
        torques: dict[str, float] = {}
        for name in targets:
            error = float(targets[name] - measured[name])
            if not np.isfinite(error):
                raise ValueError(f"wheel speed error for {name!r} must be finite")
            candidate = float(
                np.clip(
                    self.integral.get(name, 0.0) + error * dt,
                    -self.gains.integral_limit,
                    self.gains.integral_limit,
                )
            )
            unsaturated = self.gains.kp * error + self.gains.ki * candidate
            output = float(np.clip(unsaturated, -self.gains.output_limit, self.gains.output_limit))
            drives_back = (unsaturated > output and error < 0.0) or (
                unsaturated < output and error > 0.0
            )
            if unsaturated == output or drives_back:
                self.integral[name] = candidate
            torques[name] = output
        return torques


class PulseTeleop:
    """Latches 1/2/3/4 direction presses into linearly-decaying commands.

    Number keys avoid any overlap with MuJoCo viewer key handling.  Mapping is
    1 = forward, 2 = backward, 3 = turn left, 4 = turn right.  Forward drives
    the chassis -x direction: the default camera looks at the robot's rear
    (+x front legs point away from the operator), so "forward" is the near
    side and 1 drives toward the operator.

    The viewer's ``key_callback`` only delivers key-down events, so the teleop
    latches a direction for each axis and decays it linearly to zero over
    ``decay`` seconds.  Re-pressing an axis restarts its decay window.  The
    clock ``t`` is wall time (``time.monotonic()``), not simulation time.
    """

    def __init__(self) -> None:
        self._lock = Lock()
        self._vx_sign = 0.0
        self._vx_t0: float | None = None
        self._yaw_sign = 0.0
        self._yaw_t0: float | None = None

    def press(self, key: str, t: float) -> None:
        """Record a direction press for the given axis at wall time ``t``."""

        if not np.isfinite(t):
            raise ValueError("press time must be finite")
        with self._lock:
            if key == "1":  # forward (operator-visible forward, chassis -x)
                self._vx_sign, self._vx_t0 = -1.0, t
            elif key == "2":  # backward
                self._vx_sign, self._vx_t0 = 1.0, t
            elif key == "3":  # turn left
                self._yaw_sign, self._yaw_t0 = 1.0, t
            elif key == "4":  # turn right
                self._yaw_sign, self._yaw_t0 = -1.0, t

    def command(
        self,
        t: float,
        *,
        forward_speed: float,
        yaw_rate: float,
        decay: float,
    ) -> DriveCommand:
        """Return the drive command at wall time ``t`` after linear decay."""

        values = (forward_speed, yaw_rate, decay, t)
        if any(not np.isfinite(value) for value in values):
            raise ValueError("teleop command inputs must be finite")
        if forward_speed < 0.0 or yaw_rate < 0.0 or decay <= 0.0:
            raise ValueError("forward_speed/yaw_rate must be nonnegative and decay positive")
        with self._lock:
            v_x = self._axis_value(self._vx_sign, self._vx_t0, t, forward_speed, decay)
            omega_yaw = self._axis_value(self._yaw_sign, self._yaw_t0, t, yaw_rate, decay)
        return DriveCommand(v_x=v_x, omega_yaw=omega_yaw)

    @staticmethod
    def _axis_value(
        sign: float,
        t0: float | None,
        t: float,
        amplitude: float,
        decay: float,
    ) -> float:
        if t0 is None:
            return 0.0
        fraction = (t - t0) / decay
        if fraction >= 1.0:
            return 0.0
        return sign * amplitude * (1.0 - max(fraction, 0.0))

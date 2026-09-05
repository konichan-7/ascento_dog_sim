"""Wheel-speed teleop control for the four-wheeled robot, independent of MuJoCo.

The module provides four independent pieces:

- ``TeleopCommand``: a two-degree-of-freedom chassis-frame teleop command.
- ``wheel_speed_targets``: maps a teleop command to per-wheel angular speeds.
- ``WheelVelocityController``: a per-wheel PI speed controller with anti-windup.
- ``HoldTeleop``: maps held 1/2/3/4 keys to constant commands.

All quantities are SI (m, rad, s).  No physics engine is imported here.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from threading import Lock

import numpy as np


@dataclass(frozen=True)
class TeleopCommand:
    """Two-degree-of-freedom teleop command in the chassis frame.

    ``v_x`` is forward speed (m/s) along chassis +x; ``omega_yaw`` is the
    yaw rate (rad/s) about chassis +z, positive counterclockwise from above.
    """

    v_x: float
    omega_yaw: float


def wheel_speed_targets(
    command: TeleopCommand,
    wheel_radius: float,
    mounts_y: Mapping[str, float],
) -> dict[str, float]:
    """Map a teleop command to per-wheel target angular velocities (rad/s).

    Each wheel rolls only along the chassis +x axis.  The wheel-centre linear
    speed is ``v_x - omega_yaw * y`` (translation plus yaw lever arm).  In this
    model a wheel that spins positively about its +y axis rolls forward, so
    ``omega_wheel = (v_x - omega_yaw*y) / R``.
    """

    radius = float(wheel_radius)
    if not np.isfinite(radius) or radius <= 0.0:
        raise ValueError("wheel_radius must be finite and positive")
    if not np.isfinite(command.v_x) or not np.isfinite(command.omega_yaw):
        raise ValueError("teleop command must be finite")
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


class HoldTeleop:
    """Track held number keys, independent of key-repeat timing.

    1/2 command chassis +x/-x (m/s); 3/4 command yaw about +z/-z (rad/s).
    Opposite keys cancel; translation and yaw can be combined. Releasing a
    key removes its command immediately. Non-finite or negative speeds raise
    ValueError. This input mapping has no dependence on leg assembly geometry.
    """

    def __init__(self) -> None:
        self._lock = Lock()
        self._held: set[str] = set()

    def reset(self) -> None:
        """Clear held keys, e.g. on window focus loss or shutdown."""
        with self._lock:
            self._held.clear()

    def press(self, key: str) -> None:
        """Mark 1/2/3/4 held; repeats are idempotent, other keys ignored."""
        if key in ("1", "2", "3", "4"):
            with self._lock:
                self._held.add(key)

    def release(self, key: str) -> None:
        """Remove a held key; unmatched releases are harmless."""
        with self._lock:
            self._held.discard(key)

    def command(self, *, forward_speed: float, yaw_rate: float) -> TeleopCommand:
        """Return held-key velocity targets in chassis coordinates (m/s, rad/s).

        Speeds must be finite and nonnegative, otherwise raise ValueError.
        """
        if any(not np.isfinite(value) for value in (forward_speed, yaw_rate)):
            raise ValueError("teleop command inputs must be finite")
        if forward_speed < 0.0 or yaw_rate < 0.0:
            raise ValueError("forward_speed/yaw_rate must be nonnegative")
        with self._lock:
            return TeleopCommand(
                v_x=forward_speed * (("1" in self._held) - ("2" in self._held)),
                omega_yaw=yaw_rate * (("3" in self._held) - ("4" in self._held)),
            )

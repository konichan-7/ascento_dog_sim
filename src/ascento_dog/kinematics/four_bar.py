"""Planar analytical kinematics for the Ascento-style one-DoF leg."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, pi, sin

import numpy as np
from numpy.typing import NDArray

Vector2 = NDArray[np.float64]


class UnreachableTargetError(ValueError):
    """Raised when a joint or Cartesian target is outside the working branch."""


@dataclass(frozen=True)
class LegPose:
    """Linkage point positions in the A-fixed sagittal frame, in meters."""

    q: float
    a: Vector2
    b: Vector2
    c: Vector2
    d: Vector2
    e: Vector2
    phi: float
    psi: float

    def points(self) -> dict[str, Vector2]:
        """Return points A-E as a name-to-position mapping."""

        return {"A": self.a, "B": self.b, "C": self.c, "D": self.d, "E": self.e}


@dataclass(frozen=True)
class FourBarGeometry:
    """Dimensions and conventions of the one-DoF linkage.

    Lengths are meters and angles are radians. The working branch has
    ``cross(D - B, C - D) > 0``.
    """

    l1: float = 0.235
    l2: float = 0.238
    l3: float = 0.244
    l4: float = 0.10889
    l23: float = 0.057
    frame_angle: float = pi / 4.0
    q_min: float = -65.0 * pi / 180.0
    q_max: float = -15.0 * pi / 180.0
    q_nominal: float = -40.0 * pi / 180.0

    def __post_init__(self) -> None:
        lengths = (self.l1, self.l2, self.l3, self.l4, self.l23)
        if any(length <= 0.0 for length in lengths):
            raise ValueError("all linkage lengths must be positive")
        if not self.q_min < self.q_nominal < self.q_max:
            raise ValueError("q_nominal must lie strictly inside the working limits")

    @property
    def a(self) -> Vector2:
        """Fixed hip point A, in meters."""

        return np.zeros(2, dtype=float)

    @property
    def b(self) -> Vector2:
        """Fixed pin point B, in meters."""

        return self.l4 * np.array([cos(self.frame_angle), sin(self.frame_angle)])

    def forward(self, q: float, *, check_limits: bool = True) -> LegPose:
        """Compute points A-E from the absolute hip angle ``q``.

        Args:
            q: Counter-clockwise angle from +x to AD, in radians.
            check_limits: Enforce the provisional nonsingular working interval.

        Raises:
            UnreachableTargetError: If ``q`` is outside the working interval or
                the circles do not close on the selected assembly branch.
        """

        q = float(q)
        if check_limits and not self.q_min <= q <= self.q_max:
            raise UnreachableTargetError(
                f"hip angle {q:.9g} rad is outside [{self.q_min:.9g}, {self.q_max:.9g}]"
            )

        a = self.a
        b = self.b
        d = self.l2 * np.array([cos(q), sin(q)], dtype=float)
        intersections = _circle_intersections(b, self.l3, d, self.l23)
        candidates = [
            c for c in intersections if _cross_2d(d - b, c - d) > 1.0e-12
        ]
        if len(candidates) != 1:
            raise UnreachableTargetError(
                f"hip angle {q:.9g} rad does not have one nonsingular working-branch closure"
            )

        c = candidates[0]
        e = d + (self.l1 / self.l23) * (d - c)
        phi = atan2(c[1] - b[1], c[0] - b[0])
        psi = atan2(c[1] - d[1], c[0] - d[0])
        return LegPose(q=q, a=a, b=b, c=c, d=d, e=e, phi=phi, psi=psi)

    def inverse(
        self,
        wheel_position: Vector2 | tuple[float, float] | list[float],
        *,
        tolerance: float = 1.0e-8,
        check_limits: bool = True,
    ) -> float:
        """Recover the hip angle for an exactly reachable wheel-center target.

        The target is expressed in the A-fixed ``(x, z)`` frame in meters.
        Since the Cartesian workspace is one-dimensional, off-curve targets are
        rejected instead of being silently projected onto the path.
        """

        e = np.asarray(wheel_position, dtype=float)
        if e.shape != (2,) or not np.all(np.isfinite(e)):
            raise ValueError("wheel_position must contain two finite coordinates")
        if tolerance <= 0.0:
            raise ValueError("tolerance must be positive")

        solutions: list[tuple[float, float]] = []
        for d in _circle_intersections(self.a, self.l2, e, self.l1):
            c = d + (self.l23 / self.l1) * (d - e)
            closure_error = abs(float(np.linalg.norm(c - self.b)) - self.l3)
            if closure_error > tolerance:
                continue
            if _cross_2d(d - self.b, c - d) <= 1.0e-12:
                continue
            q = atan2(d[1], d[0])
            if check_limits and not self.q_min - tolerance <= q <= self.q_max + tolerance:
                continue
            try:
                reconstructed = self.forward(q, check_limits=check_limits)
            except UnreachableTargetError:
                continue
            target_error = float(np.linalg.norm(reconstructed.e - e))
            if target_error <= tolerance:
                solutions.append((target_error, q))

        if not solutions:
            raise UnreachableTargetError(
                "wheel target is not on the selected one-DoF linkage path within "
                f"{tolerance:g} m"
            )
        solutions.sort(key=lambda item: item[0])
        return solutions[0][1]

    def inverse_height(
        self,
        z: float,
        *,
        tolerance: float = 1.0e-10,
        max_iterations: int = 80,
    ) -> float:
        """Find ``q`` for a reachable wheel height ``z`` on the working branch."""

        if not np.isfinite(z):
            raise ValueError("z must be finite")
        if tolerance <= 0.0 or max_iterations <= 0:
            raise ValueError("tolerance and max_iterations must be positive")

        lo, hi = self.q_min, self.q_max
        f_lo = self.forward(lo).e[1] - z
        f_hi = self.forward(hi).e[1] - z
        if abs(f_lo) <= tolerance:
            return lo
        if abs(f_hi) <= tolerance:
            return hi
        if f_lo * f_hi > 0.0:
            heights = sorted((self.forward(lo).e[1], self.forward(hi).e[1]))
            raise UnreachableTargetError(
                f"wheel height {z:.9g} m is outside [{heights[0]:.9g}, {heights[1]:.9g}]"
            )

        for _ in range(max_iterations):
            mid = 0.5 * (lo + hi)
            f_mid = self.forward(mid).e[1] - z
            if abs(f_mid) <= tolerance or 0.5 * (hi - lo) <= tolerance:
                return mid
            if f_lo * f_mid <= 0.0:
                hi = mid
            else:
                lo, f_lo = mid, f_mid
        return 0.5 * (lo + hi)

    def wheel_jacobian(self, q: float, *, step: float = 1.0e-6) -> Vector2:
        """Return ``dE/dq`` in meters per radian using a centered difference."""

        if step <= 0.0:
            raise ValueError("step must be positive")
        if not self.q_min + step <= q <= self.q_max - step:
            raise UnreachableTargetError("q must be at least one finite-difference step from a limit")
        return (self.forward(q + step).e - self.forward(q - step).e) / (2.0 * step)

    def bar_residuals(self, pose: LegPose) -> dict[str, float]:
        """Return signed bar-length residuals in meters for a computed pose."""

        return {
            "AB": float(np.linalg.norm(pose.b - pose.a)) - self.l4,
            "AD": float(np.linalg.norm(pose.d - pose.a)) - self.l2,
            "BC": float(np.linalg.norm(pose.c - pose.b)) - self.l3,
            "CD": float(np.linalg.norm(pose.c - pose.d)) - self.l23,
            "DE": float(np.linalg.norm(pose.e - pose.d)) - self.l1,
        }


DEFAULT_GEOMETRY = FourBarGeometry()


def _cross_2d(first: Vector2, second: Vector2) -> float:
    return float(first[0] * second[1] - first[1] * second[0])


def _circle_intersections(
    center_0: Vector2,
    radius_0: float,
    center_1: Vector2,
    radius_1: float,
    *,
    tolerance: float = 1.0e-12,
) -> tuple[Vector2, ...]:
    """Return zero, one, or two intersections between planar circles."""

    delta = center_1 - center_0
    distance = float(np.linalg.norm(delta))
    if distance <= tolerance:
        return ()
    if distance > radius_0 + radius_1 + tolerance:
        return ()
    if distance < abs(radius_0 - radius_1) - tolerance:
        return ()

    along = (radius_0**2 - radius_1**2 + distance**2) / (2.0 * distance)
    height_squared = radius_0**2 - along**2
    if height_squared < -tolerance:
        return ()
    height = float(np.sqrt(max(0.0, height_squared)))
    direction = delta / distance
    midpoint = center_0 + along * direction
    if height <= tolerance:
        return (midpoint,)
    perpendicular = np.array([-direction[1], direction[0]])
    return midpoint + height * perpendicular, midpoint - height * perpendicular

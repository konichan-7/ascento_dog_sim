"""Virtual-model control for the four-wheeled Ascento-style robot.

The controller is independent of MuJoCo.  It combines gravity compensation,
height PID feedback, roll/pitch attitude feedback, bounded vertical-force
allocation, and analytical Jacobian-transpose force-to-torque mapping.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import product
from math import inf
from typing import Mapping

import numpy as np
from numpy.typing import NDArray

from ascento_dog.kinematics import DEFAULT_GEOMETRY, FourBarGeometry

Vector3 = NDArray[np.float64]
Matrix3 = NDArray[np.float64]


@dataclass(frozen=True)
class PIDGains:
    """Height PID gains and limits in SI units.

    ``kp``, ``ki`` and ``kd`` have units N/m, N/(m*s), and N*s/m.  Limits are
    applied to the integral state in m*s and to the PID thrust in N.
    """

    kp: float
    ki: float
    kd: float
    integral_limit: float = inf
    output_limit: float = inf

    def __post_init__(self) -> None:
        values = (self.kp, self.ki, self.kd)
        if any(not np.isfinite(value) or value < 0.0 for value in values):
            raise ValueError("PID gains must be finite and nonnegative")
        if self.integral_limit <= 0.0 or self.output_limit <= 0.0:
            raise ValueError("PID limits must be positive")


@dataclass
class HeightPID:
    """Height PID with derivative-on-measurement and conditional anti-windup."""

    gains: PIDGains
    integral: float = 0.0

    def reset(self) -> None:
        """Clear the integral state."""

        self.integral = 0.0

    def update(
        self,
        desired_height: float,
        measured_height: float,
        vertical_velocity: float,
        dt: float,
    ) -> float:
        """Return the virtual thrust increment in N.

        Heights and velocity are world-frame meters and m/s.  Positive output
        increases the total upward support force above gravity compensation.
        """

        inputs = (desired_height, measured_height, vertical_velocity, dt)
        if any(not np.isfinite(value) for value in inputs) or dt <= 0.0:
            raise ValueError("PID inputs must be finite and dt must be positive")

        error = desired_height - measured_height
        candidate_integral = float(
            np.clip(
                self.integral + error * dt,
                -self.gains.integral_limit,
                self.gains.integral_limit,
            )
        )
        unsaturated = (
            self.gains.kp * error
            + self.gains.ki * candidate_integral
            - self.gains.kd * vertical_velocity
        )
        output = float(
            np.clip(unsaturated, -self.gains.output_limit, self.gains.output_limit)
        )

        # Integrate when unsaturated, or when the current error moves a
        # saturated output back toward its admissible interval.
        drives_back = (unsaturated > output and error < 0.0) or (
            unsaturated < output and error > 0.0
        )
        if unsaturated == output or drives_back:
            self.integral = candidate_integral
        return output


@dataclass(frozen=True)
class AttitudeGains:
    """Roll/pitch PD gains in N*m/rad and N*m*s/rad."""

    roll_kp: float
    roll_kd: float
    pitch_kp: float
    pitch_kd: float

    def __post_init__(self) -> None:
        values = (self.roll_kp, self.roll_kd, self.pitch_kp, self.pitch_kd)
        if any(not np.isfinite(value) or value < 0.0 for value in values):
            raise ValueError("attitude gains must be finite and nonnegative")


@dataclass(frozen=True)
class LegMount:
    """Leg-frame pose relative to the chassis frame.

    ``position_body`` is in meters and ``rotation_body_from_leg`` maps a vector
    from the analytical leg frame into the chassis frame.
    """

    position_body: Vector3
    rotation_body_from_leg: Matrix3 = field(default_factory=lambda: np.eye(3))

    def __post_init__(self) -> None:
        position = np.asarray(self.position_body, dtype=float)
        rotation = np.asarray(self.rotation_body_from_leg, dtype=float)
        if position.shape != (3,) or rotation.shape != (3, 3):
            raise ValueError("mount position/rotation must have shapes (3,) and (3,3)")
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
            raise ValueError("mount pose must be finite")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-9):
            raise ValueError("mount rotation must be orthonormal")
        object.__setattr__(self, "position_body", position.copy())
        object.__setattr__(self, "rotation_body_from_leg", rotation.copy())


@dataclass(frozen=True)
class VMCState:
    """State consumed by the controller, expressed in SI units.

    ``body_to_world`` maps chassis-frame vectors into the world frame.
    ``angular_velocity_body`` is ordered ``[roll, pitch, yaw]`` about body axes.
    Leg angles are the absolute analytical angles ``q`` in radians.
    """

    height: float
    vertical_velocity: float
    roll: float
    pitch: float
    angular_velocity_body: Vector3
    body_to_world: Matrix3
    leg_angles: Mapping[str, float]
    yaw: float = 0.0
    center_of_mass_body: Vector3 = field(default_factory=lambda: np.zeros(3))


@dataclass(frozen=True)
class VMCCommand:
    """Controller result with forces in N and hip torques in N*m."""

    gravity_compensation: float
    height_pid_thrust: float
    desired_total_force: float
    desired_body_torque: NDArray[np.float64]
    leg_forces: dict[str, float]
    hip_torques: dict[str, float]
    achieved_wrench: NDArray[np.float64]
    allocation_residual: NDArray[np.float64]


def vertical_force_allocation_matrix(
    contact_points_body: NDArray[np.float64], body_to_world: Matrix3
) -> NDArray[np.float64]:
    """Build the vertical-force allocation matrix.

    Contact points are relative to the chassis center of mass in the chassis
    frame, in meters.  Each column maps one nonnegative world-up force to
    ``[Fz_world, tau_roll_body, tau_pitch_body]``.
    """

    points = np.asarray(contact_points_body, dtype=float)
    rotation = np.asarray(body_to_world, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("contact_points_body must have shape (N, 3)")
    if rotation.shape != (3, 3):
        raise ValueError("body_to_world must have shape (3, 3)")
    if not np.all(np.isfinite(points)) or not np.all(np.isfinite(rotation)):
        raise ValueError("allocation inputs must be finite")

    world_up_body = rotation.T @ np.array([0.0, 0.0, 1.0])
    moments = np.cross(points, world_up_body)
    return np.vstack((np.ones(points.shape[0]), moments[:, 0], moments[:, 1]))


def allocate_vertical_forces(
    desired_wrench: NDArray[np.float64],
    contact_points_body: NDArray[np.float64],
    body_to_world: Matrix3,
    *,
    minimum_force: float | NDArray[np.float64] = 0.0,
    maximum_force: float | NDArray[np.float64] = inf,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Allocate bounded vertical wheel forces using a small active-set search.

    The desired wrench is ``[Fz_world, tau_roll_body, tau_pitch_body]``.  The
    returned tuple contains the force vector and achieved wrench.  If force
    limits make the request infeasible, the solution minimizes a normalized
    wrench residual while respecting every bound.
    """

    matrix = vertical_force_allocation_matrix(contact_points_body, body_to_world)
    target = np.asarray(desired_wrench, dtype=float)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError("desired_wrench must contain three finite values")
    count = matrix.shape[1]
    lower = np.broadcast_to(np.asarray(minimum_force, dtype=float), (count,)).copy()
    upper = np.broadcast_to(np.asarray(maximum_force, dtype=float), (count,)).copy()
    if np.any(np.isnan(lower)) or np.any(np.isnan(upper)):
        raise ValueError("force bounds may not be NaN")
    if np.any(lower > upper):
        raise ValueError("minimum_force must not exceed maximum_force")

    reference = np.full(count, max(0.0, target[0]) / count)
    reference = np.clip(reference, lower, upper)
    lever_scale = max(float(np.max(np.linalg.norm(contact_points_body[:, :2], axis=1))), 1e-6)
    weights = np.diag([1.0, 1.0 / lever_scale, 1.0 / lever_scale])

    best_force: NDArray[np.float64] | None = None
    best_cost = inf
    # Per leg: -1 fixed at lower bound, 0 free, +1 fixed at upper bound.
    for states in product((-1, 0, 1), repeat=count):
        if any(state > 0 and not np.isfinite(upper[index]) for index, state in enumerate(states)):
            continue
        force = reference.copy()
        fixed = [index for index, state in enumerate(states) if state != 0]
        free = [index for index, state in enumerate(states) if state == 0]
        for index in fixed:
            force[index] = lower[index] if states[index] < 0 else upper[index]

        if free:
            free_matrix = matrix[:, free]
            residual_target = target - matrix[:, fixed] @ force[fixed] if fixed else target
            reference_free = reference[free]
            correction = np.linalg.pinv(weights @ free_matrix) @ (
                weights @ (residual_target - free_matrix @ reference_free)
            )
            force[free] = reference_free + correction
        if np.any(force < lower - 1.0e-9) or np.any(force > upper + 1.0e-9):
            continue
        force = np.clip(force, lower, upper)
        wrench_error = weights @ (matrix @ force - target)
        cost = float(wrench_error @ wrench_error + 1.0e-12 * np.sum((force - reference) ** 2))
        if cost < best_cost:
            best_cost = cost
            best_force = force.copy()

    if best_force is None:  # Only possible for malformed infinite bounds.
        raise RuntimeError("bounded force allocation failed")
    return best_force, matrix @ best_force


class QuadrupedVMC:
    """Gravity, height, and attitude VMC for four one-DoF legs."""

    def __init__(
        self,
        *,
        mass: float,
        mounts: Mapping[str, LegMount],
        height_pid: HeightPID,
        attitude_gains: AttitudeGains,
        geometry: FourBarGeometry = DEFAULT_GEOMETRY,
        gravity: float = 9.81,
        minimum_leg_force: float = 0.0,
        maximum_leg_force: float = inf,
        maximum_hip_torque: float = inf,
    ) -> None:
        if not np.isfinite(mass) or mass <= 0.0:
            raise ValueError("mass must be finite and positive")
        if not np.isfinite(gravity) or gravity <= 0.0:
            raise ValueError("gravity must be finite and positive")
        if len(mounts) != 4:
            raise ValueError("exactly four leg mounts are required")
        if minimum_leg_force < 0.0 or minimum_leg_force > maximum_leg_force:
            raise ValueError("invalid leg-force limits")
        if maximum_hip_torque <= 0.0:
            raise ValueError("maximum_hip_torque must be positive")
        self.mass = float(mass)
        self.mounts = dict(mounts)
        self.height_pid = height_pid
        self.attitude_gains = attitude_gains
        self.geometry = geometry
        self.gravity = float(gravity)
        self.minimum_leg_force = float(minimum_leg_force)
        self.maximum_leg_force = float(maximum_leg_force)
        self.maximum_hip_torque = float(maximum_hip_torque)

    def reset(self) -> None:
        """Reset all controller memory."""

        self.height_pid.reset()

    def compute(
        self, state: VMCState, *, desired_height: float, dt: float
    ) -> VMCCommand:
        """Compute bounded wheel forces and hip torques for one control step."""

        if set(state.leg_angles) != set(self.mounts):
            raise ValueError("state leg names must match configured mounts")
        rotation = np.asarray(state.body_to_world, dtype=float)
        angular_velocity = np.asarray(state.angular_velocity_body, dtype=float)
        center_of_mass = np.asarray(state.center_of_mass_body, dtype=float)
        if (
            rotation.shape != (3, 3)
            or angular_velocity.shape != (3,)
            or center_of_mass.shape != (3,)
        ):
            raise ValueError("invalid orientation, velocity, or center-of-mass shape")

        gravity_force = self.mass * self.gravity
        height_thrust = self.height_pid.update(
            desired_height, state.height, state.vertical_velocity, dt
        )
        desired_total = max(0.0, gravity_force + height_thrust)
        roll_torque = (
            -self.attitude_gains.roll_kp * state.roll
            - self.attitude_gains.roll_kd * angular_velocity[0]
        )
        pitch_torque = (
            -self.attitude_gains.pitch_kp * state.pitch
            - self.attitude_gains.pitch_kd * angular_velocity[1]
        )
        desired_wrench = np.array([desired_total, roll_torque, pitch_torque])

        names = tuple(self.mounts)
        contact_points: list[Vector3] = []
        for name in names:
            pose = self.geometry.forward(float(state.leg_angles[name]))
            wheel_leg = np.array([pose.e[0], 0.0, pose.e[1]])
            mount = self.mounts[name]
            contact_points.append(
                mount.position_body
                + mount.rotation_body_from_leg @ wheel_leg
                - center_of_mass
            )
        points = np.asarray(contact_points)
        forces, achieved = allocate_vertical_forces(
            desired_wrench,
            points,
            rotation,
            minimum_force=self.minimum_leg_force,
            maximum_force=self.maximum_leg_force,
        )

        world_up = np.array([0.0, 0.0, 1.0])
        hip_torques: dict[str, float] = {}
        for index, name in enumerate(names):
            q = float(state.leg_angles[name])
            jacobian_xz = self.geometry.wheel_jacobian(q)
            jacobian_leg = np.array([jacobian_xz[0], 0.0, jacobian_xz[1]])
            jacobian_world = (
                rotation @ self.mounts[name].rotation_body_from_leg @ jacobian_leg
            )
            # Static virtual-work balance: tau + J^T F_ground = 0.
            torque = -forces[index] * float(world_up @ jacobian_world)
            hip_torques[name] = float(
                np.clip(torque, -self.maximum_hip_torque, self.maximum_hip_torque)
            )

        return VMCCommand(
            gravity_compensation=gravity_force,
            height_pid_thrust=height_thrust,
            desired_total_force=desired_total,
            desired_body_torque=np.array([roll_torque, pitch_torque]),
            leg_forces=dict(zip(names, forces, strict=True)),
            hip_torques=hip_torques,
            achieved_wrench=achieved,
            allocation_residual=achieved - desired_wrench,
        )

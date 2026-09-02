"""Phased step-crossing control for the four-wheeled, four-legged robot.

The controller drives the robot forward onto a raised platform and targets
a level chassis.  Most phases use a pitch target of 0; the rear climb uses
symmetric joint targets and geometric locking instead of active pitch PD.
The front climb keeps all legs in VMC force control but caps the front-leg
support force so wheel-against-riser rolling friction can lift the front
axle.  The rear climb switches all legs to joint-space position control:
the front legs lock at their shortest pose while the rear legs follow a
monotonic effective-length target.  The other phases use bounded vertical-force
allocation mapped to hip torques by the analytical Jacobian transpose.

Phases (world +x is the travel direction, the riser faces -x):

1. ``APPROACH``    all legs stance; level body at ``stance_height``
   (= step top + wheel radius + shortest-leg drop, i.e. the nominal
   q=-40 deg stance on flat ground); drive toward the riser.
2. ``FRONT_CLIMB`` all legs remain in VMC force control; the front-leg
   force cap reduces downward load while wheel drive and riser friction
   lift the front wheels.  The front legs retract naturally as the wheels
   rise.  Body stays level.
3. ``STRADDLE``    front wheels on the platform, front legs shortest,
   rear legs stance; body level; drive until the rear wheels touch the
   riser.
4. ``REAR_CLIMB``  front legs hold shortest (position), rear legs retract
   on the effective-length schedule, dragging the rear wheels up the face; the front
   wheels drive/press on the platform.  Symmetric targets and geometric
   locking are intended to keep the body level without active pitch PD.
5. ``EXTEND``      all four wheels on the platform; stop the wheels and
   interpolate all four hip targets to the nominal pose.  This geometric
   recovery levels the chassis before VMC is restored.
6. ``DONE``        hold pose on the platform.

Sign conventions (identical to :mod:`ascento_dog.control.vmc`):

- Pitch is the body +y (left) axis Euler angle; positive pitch tips the
  nose DOWN.  The crossing keeps the body level (pitch target 0).
- ``height`` is the chassis-center world z in meters; leg angles ``q`` are
  absolute analytical hip angles in radians; forces are newtons and hip
  torques N*m.  The ground under the approach side is at ``z = 0``.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import Enum
from math import inf

import numpy as np

from ascento_dog.control.vmc import (
    HeightPID,
    LegMount,
    PIDGains,
    VMCState,
    allocate_vertical_forces,
)
from ascento_dog.control.wheel_speed import (
    TeleopCommand,
    WheelSpeedGains,
    WheelVelocityController,
    wheel_speed_targets,
)
from ascento_dog.kinematics import DEFAULT_GEOMETRY, FourBarGeometry

FRONT_LEGS = ("front_left", "front_right")
REAR_LEGS = ("rear_left", "rear_right")
ALL_LEGS = FRONT_LEGS + REAR_LEGS


class CrossingPhase(Enum):
    """Sequential phases of one step-crossing attempt."""

    APPROACH = "approach"
    FRONT_CLIMB = "front_climb"
    STRADDLE = "straddle"
    REAR_CLIMB = "rear_climb"
    EXTEND = "extend"
    DONE = "done"


@dataclass(frozen=True)
class WheelObservation:
    """World-frame wheel-center observation in meters.

    ``x`` is the travel-direction coordinate of the wheel center and ``z``
    its height, both in the world frame.
    """

    x: float
    z: float


@dataclass(frozen=True)
class CrossingParameters:
    """Step geometry, pose targets, gains, and timeouts (SI units).

    Heights are chassis-center world z targets.  ``stance_height`` is the
    level-body height during the crossing and should equal
    ``step_height + wheel_radius + shortest_leg_drop`` (the front legs can
    then reach the platform top exactly at their shortest length, keeping
    the chassis level); ``extend_height`` is the nominal stance height on
    top of the platform.  All gains remain illustrative until actuator
    identification.
    """

    step_height: float = 0.20
    step_face_x: float = 0.9
    wheel_radius: float = 0.065
    wheelbase: float = 0.48
    contact_margin: float = 0.005
    edge_clearance: float = 0.04
    stance_height: float = 0.376206
    extend_height: float = 0.5693091272
    extend_duration: float = 1.5
    extend_settle: float = 0.02
    extend_attitude_settle: float = np.deg2rad(1.0)
    forward_speed: float = 0.25
    front_press_offset: float = 0.05
    rear_press_offset: float = 0.15
    rear_climb_speed: float = 0.45
    front_climbing_cap: float = 70.0
    rear_retraction_rate: float = 0.16
    air_kp: float = 300.0
    air_kd: float = 12.0
    height_kp: float = 1000.0
    height_ki: float = 200.0
    height_kd: float = 260.0
    height_integral_limit: float = 0.15
    height_thrust_limit: float = 180.0
    roll_kp: float = 180.0
    roll_kd: float = 28.0
    pitch_kp: float = 360.0
    pitch_kd: float = 80.0
    minimum_leg_force: float = 25.0
    maximum_leg_force: float = 120.0
    maximum_hip_torque: float = 40.0
    wheel_kp: float = 0.8
    wheel_ki: float = 0.3
    wheel_integral_limit: float = 3.0
    wheel_output_limit: float = 20.0
    phase_timeouts: Mapping[CrossingPhase, float] = field(
        default_factory=lambda: {
            CrossingPhase.APPROACH: 20.0,
            CrossingPhase.FRONT_CLIMB: 15.0,
            CrossingPhase.STRADDLE: 20.0,
            CrossingPhase.REAR_CLIMB: 15.0,
            CrossingPhase.EXTEND: 8.0,
            CrossingPhase.DONE: inf,
        }
    )

    def __post_init__(self) -> None:
        if self.step_height <= 0.0 or self.wheel_radius <= 0.0:
            raise ValueError("step height and wheel radius must be positive")
        if self.wheelbase <= 0.0:
            raise ValueError("wheelbase must be positive")
        if self.stance_height <= 0.0 or self.extend_height <= 0.0:
            raise ValueError("stance/extend heights must be positive")
        if self.front_press_offset < 0.0 or self.rear_press_offset < 0.0:
            raise ValueError("press offsets must be nonnegative")
        if self.rear_climb_speed < 0.0:
            raise ValueError("rear climb speed must be nonnegative")
        if self.front_climbing_cap <= 0.0:
            raise ValueError("front_climbing_cap must be positive")
        if self.rear_retraction_rate <= 0.0:
            raise ValueError("rear_retraction_rate must be positive")
        if self.air_kp <= 0.0 or self.air_kd < 0.0:
            raise ValueError("air-leg gains must be positive kp and nonnegative kd")
        if self.forward_speed < 0.0:
            raise ValueError("forward speed must be nonnegative")
        if self.extend_duration <= 0.0 or self.extend_settle <= 0.0:
            raise ValueError("extend duration and settle must be positive")
        if self.extend_attitude_settle <= 0.0:
            raise ValueError("extend attitude settle must be positive")


@dataclass(frozen=True)
class CrossingCommand:
    """One control-step output, forces in N and torques in N*m."""

    phase: CrossingPhase
    hip_torques: dict[str, float]
    wheel_torques: dict[str, float]
    leg_forces: dict[str, float]
    desired_height: float
    desired_pitch: float
    done: bool
    failed: bool
    failure_reason: str = ""


class StepCrossingController:
    """Hybrid force/position step crossing for four one-DoF legs.

    The controller is independent of MuJoCo.  It consumes the same
    :class:`~ascento_dog.control.vmc.VMCState` as the VMC plus per-wheel
    world observations and hip joint rates, and produces hip and wheel
    torques for one simulation step.

    Stance support and the front climb use vertical-force allocation mapped
    to hip torques by the analytical Jacobian transpose.  Only the rear
    climb switches all four legs to joint-space position control: the front
    axle locks at minimum leg length and the rear axle follows a scheduled
    wheel lift height.
    """

    def __init__(
        self,
        *,
        mass: float,
        mounts: Mapping[str, LegMount],
        parameters: CrossingParameters,
        geometry: FourBarGeometry = DEFAULT_GEOMETRY,
        gravity: float = 9.81,
    ) -> None:
        if not np.isfinite(mass) or mass <= 0.0:
            raise ValueError("mass must be finite and positive")
        if set(mounts) != set(ALL_LEGS):
            raise ValueError("mounts must contain exactly the four legs")
        self.mass = float(mass)
        self.mounts = dict(mounts)
        self.parameters = parameters
        self.geometry = geometry
        self.gravity = float(gravity)
        self.height_pid = HeightPID(
            PIDGains(
                parameters.height_kp,
                parameters.height_ki,
                parameters.height_kd,
                integral_limit=parameters.height_integral_limit,
                output_limit=parameters.height_thrust_limit,
            )
        )
        self.wheel_controller = WheelVelocityController(
            WheelSpeedGains(
                parameters.wheel_kp,
                parameters.wheel_ki,
                integral_limit=parameters.wheel_integral_limit,
                output_limit=parameters.wheel_output_limit,
            )
        )
        self.phase = CrossingPhase.APPROACH
        self.phase_time = 0.0
        self.failed = False
        self.failure_reason = ""
        self._extend_start_height = parameters.extend_height
        self._extend_start_angles: dict[str, float] | None = None
        self._rear_leg_length_target: float | None = None

    def reset(self) -> None:
        """Reset the state machine and all controller memory."""

        self.height_pid.reset()
        self.wheel_controller.reset()
        self.phase = CrossingPhase.APPROACH
        self.phase_time = 0.0
        self.failed = False
        self.failure_reason = ""
        self._extend_start_angles = None
        self._rear_leg_length_target = None

    def q_for_leg_length(self, length: float) -> float:
        """Return the hip angle for effective length ``|AE|`` in meters.

        The target is clamped to the selected working branch endpoints.
        """

        if not np.isfinite(length):
            raise ValueError("leg length must be finite")

        def residual(q: float) -> float:
            return float(np.linalg.norm(self.geometry.forward(q, check_limits=False).e)) - length

        lo = self.geometry.q_min
        hi = self.geometry.q_max
        residual_lo = residual(lo)
        residual_hi = residual(hi)
        if residual_lo <= 0.0:
            return lo
        if residual_hi >= 0.0:
            return hi
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            residual_mid = residual(mid)
            if abs(residual_mid) <= 1.0e-10:
                return mid
            if residual_mid > 0.0:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    @property
    def rear_leg_length_target(self) -> float | None:
        """Current rear-leg effective-length target ``|AE|`` in meters."""

        return self._rear_leg_length_target

    def update(
        self,
        state: VMCState,
        wheels: Mapping[str, WheelObservation],
        *,
        wheel_velocities: Mapping[str, float],
        leg_rates: Mapping[str, float],
        dt: float,
    ) -> CrossingCommand:
        """Advance the state machine one step and return the torques to apply.

        ``wheels`` holds one :class:`WheelObservation` per leg,
        ``wheel_velocities`` the spin rates in rad/s, and ``leg_rates`` the
        hip joint rates in rad/s.
        """

        if set(wheels) != set(ALL_LEGS):
            raise ValueError("wheels must contain exactly the four legs")
        if set(leg_rates) != set(ALL_LEGS):
            raise ValueError("leg_rates must contain exactly the four legs")
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")

        self.phase_time += dt
        self._advance_phase(state, wheels)
        if self.phase == CrossingPhase.REAR_CLIMB:
            return self._rear_climb_command(state, wheels, wheel_velocities, leg_rates, dt)
        if self.phase == CrossingPhase.EXTEND:
            return self._extend_command(state, wheel_velocities, leg_rates, dt)

        height_target, pitch_target, speed = self._phase_targets(state, wheels)

        rotation = np.asarray(state.body_to_world, dtype=float)
        gravity_force = self.mass * self.gravity
        height_thrust = self.height_pid.update(
            height_target, state.height, state.vertical_velocity, dt
        )
        desired_total = max(0.0, gravity_force + height_thrust)

        roll_torque = -self.parameters.roll_kp * state.roll - self.parameters.roll_kd * float(
            state.angular_velocity_body[0]
        )
        pitch_torque = -self.parameters.pitch_kp * (
            state.pitch - pitch_target
        ) - self.parameters.pitch_kd * float(state.angular_velocity_body[1])
        desired_wrench = np.array([desired_total, roll_torque, pitch_torque])

        center_of_mass = np.asarray(state.center_of_mass_body, dtype=float)
        points = []
        lower_bounds = []
        upper_bounds = []
        for name in ALL_LEGS:
            q = self._clamped_angle(float(state.leg_angles[name]))
            pose = self.geometry.forward(q, check_limits=False)
            wheel_body = np.array([pose.e[0], 0.0, pose.e[1]])
            mount = self.mounts[name]
            points.append(
                mount.position_body + mount.rotation_body_from_leg @ wheel_body - center_of_mass
            )
            lower_bounds.append(self.parameters.minimum_leg_force)
            upper_bounds.append(self._leg_force_cap(name))
        forces, _achieved = allocate_vertical_forces(
            desired_wrench,
            np.asarray(points),
            rotation,
            minimum_force=np.asarray(lower_bounds),
            maximum_force=np.asarray(upper_bounds),
        )

        hip_torques: dict[str, float] = {}
        world_up = np.array([0.0, 0.0, 1.0])
        for index, name in enumerate(ALL_LEGS):
            q = self._clamped_angle(float(state.leg_angles[name]))
            jacobian_xz = self.geometry.wheel_jacobian(q)
            jacobian_leg = np.array([jacobian_xz[0], 0.0, jacobian_xz[1]])
            jacobian_world = rotation @ self.mounts[name].rotation_body_from_leg @ jacobian_leg
            torque = -float(forces[index]) * float(world_up @ jacobian_world)
            hip_torques[name] = float(
                np.clip(
                    torque,
                    -self.parameters.maximum_hip_torque,
                    self.parameters.maximum_hip_torque,
                )
            )

        wheel_torques = self._wheel_torques(speed, wheel_velocities, dt)

        return CrossingCommand(
            phase=self.phase,
            hip_torques=hip_torques,
            wheel_torques=wheel_torques,
            leg_forces={name: float(forces[index]) for index, name in enumerate(ALL_LEGS)},
            desired_height=height_target,
            desired_pitch=pitch_target,
            done=self.phase == CrossingPhase.DONE,
            failed=self.failed,
            failure_reason=self.failure_reason,
        )

    def _advance_phase(self, state: VMCState, wheels: Mapping[str, WheelObservation]) -> None:
        """Apply phase-transition and timeout rules."""

        timeout = self.parameters.phase_timeouts.get(self.phase, inf)
        if self.phase_time > timeout:
            self.failed = True
            self.failure_reason = f"phase {self.phase.value} exceeded {timeout:.1f} s"
            return

        front = [wheels[name] for name in FRONT_LEGS]
        rear = [wheels[name] for name in REAR_LEGS]
        p = self.parameters
        face = p.step_face_x
        top = p.step_height + p.wheel_radius

        if self.phase == CrossingPhase.APPROACH:
            if min(w.x for w in front) >= face - p.wheel_radius - p.contact_margin:
                self._enter(CrossingPhase.FRONT_CLIMB, wheels)
        elif self.phase == CrossingPhase.FRONT_CLIMB:
            if (
                min(w.x for w in front) >= face + p.edge_clearance
                and min(w.z for w in front) >= top - p.contact_margin
            ):
                self._enter(CrossingPhase.STRADDLE)
        elif self.phase == CrossingPhase.STRADDLE:
            if min(w.x for w in rear) >= face - p.wheel_radius - p.contact_margin:
                self._enter(CrossingPhase.REAR_CLIMB, wheels)
        elif self.phase == CrossingPhase.REAR_CLIMB:
            if (
                min(w.x for w in rear) >= face + p.edge_clearance
                and min(w.z for w in rear) >= top - p.contact_margin
            ):
                self._enter(CrossingPhase.EXTEND)
        elif self.phase == CrossingPhase.EXTEND:
            height_settled = abs(state.height - p.extend_height) <= p.extend_settle
            attitude_settled = max(abs(state.roll), abs(state.pitch)) <= p.extend_attitude_settle
            if self.phase_time >= p.extend_duration and height_settled and attitude_settled:
                self._enter(CrossingPhase.DONE)

    def _enter(
        self, phase: CrossingPhase, wheels: Mapping[str, WheelObservation] | None = None
    ) -> None:
        """Switch to ``phase`` and reset per-phase bookkeeping."""

        self.phase = phase
        self.phase_time = 0.0
        if phase == CrossingPhase.EXTEND:
            self._extend_start_height = None  # 由 _phase_targets 首次调用记录
            self._extend_start_angles = None
        if wheels is not None:
            if phase == CrossingPhase.REAR_CLIMB:
                self._rear_leg_length_target = None

    def _phase_targets(
        self, state: VMCState, wheels: Mapping[str, WheelObservation]
    ) -> tuple[float, float, tuple[float, float]]:
        """Return ``(height_target, pitch_target, (front_speed, rear_speed))``.

        本辅助函数处理的阶段均返回 pitch 目标 0；REAR_CLIMB 走独立位置伺服分支。
        """

        p = self.parameters
        if self.phase == CrossingPhase.DONE:
            return p.extend_height, 0.0, (0.0, 0.0)
        speed = p.forward_speed
        if self.phase == CrossingPhase.FRONT_CLIMB:
            # 前爬：前后轮都保持较强自转与顶压（前轮沿立面爬升）。
            speed += p.front_press_offset
            return p.stance_height, 0.0, (speed, speed)
        return p.stance_height, 0.0, (p.forward_speed, p.forward_speed)

    def _leg_force_cap(self, name: str) -> float:
        """Return the per-leg stance force upper bound in the current phase.

        前爬阶段，前腿（爬升轴）支撑力被压到力帽以下：轮子抵住立面的
        滚动摩擦力与顶压法向力成正比，只有摩擦力超过腿部下压力时，
        车轮才能沿立面向上爬。其余阶段取满最大支撑力。
        """

        p = self.parameters
        if self.phase == CrossingPhase.FRONT_CLIMB and name in FRONT_LEGS:
            return min(p.maximum_leg_force, p.front_climbing_cap)
        return p.maximum_leg_force

    def _rear_climb_command(
        self,
        state: VMCState,
        wheels: Mapping[str, WheelObservation],
        wheel_velocities: Mapping[str, float],
        leg_rates: Mapping[str, float],
        dt: float,
    ) -> CrossingCommand:
        """后爬：四腿位置控制，前腿锁最短、后腿按有效腿长轨迹收缩。

        前腿锁在最短长度（轮心与安装座距离最小），俯仰被几何钉死；
        后腿收缩把后轮沿立面拖上去；前轮在台面打滑顶压，后轮小自转
        维持齿轮方向。
        """

        p = self.parameters
        if self._rear_leg_length_target is None:
            rear_lengths = []
            for name in REAR_LEGS:
                q = self._clamped_angle(float(state.leg_angles[name]))
                rear_lengths.append(float(np.linalg.norm(self.geometry.forward(q).e)))
            self._rear_leg_length_target = float(np.mean(rear_lengths))
        minimum_length = float(np.linalg.norm(self.geometry.forward(self.geometry.q_max).e))
        self._rear_leg_length_target = max(
            minimum_length,
            self._rear_leg_length_target - p.rear_retraction_rate * dt,
        )
        rear_q_target = self.q_for_leg_length(self._rear_leg_length_target)
        hip_torques: dict[str, float] = {}
        for name in ALL_LEGS:
            q = self._clamped_angle(float(state.leg_angles[name]))
            if name in REAR_LEGS:
                q_target = rear_q_target
            else:
                q_target = self.geometry.q_max  # 最短：收缩到位
            hip_torques[name] = self._servo_torque(q, q_target, leg_rates[name])

        speed = (p.forward_speed + p.rear_press_offset, p.rear_climb_speed)
        wheel_torques = self._wheel_torques(speed, wheel_velocities, dt)
        return CrossingCommand(
            phase=self.phase,
            hip_torques=hip_torques,
            wheel_torques=wheel_torques,
            leg_forces={name: 0.0 for name in ALL_LEGS},
            desired_height=p.stance_height,
            desired_pitch=0.0,
            done=False,
            failed=self.failed,
            failure_reason=self.failure_reason,
        )

    def _extend_command(
        self,
        state: VMCState,
        wheel_velocities: Mapping[str, float],
        leg_rates: Mapping[str, float],
        dt: float,
    ) -> CrossingCommand:
        """台上恢复：停车并将四腿关节目标平滑插值到标称姿态。"""

        p = self.parameters
        if self._extend_start_height is None:
            self._extend_start_height = float(state.height)
        if self._extend_start_angles is None:
            self._extend_start_angles = {
                name: self._clamped_angle(float(state.leg_angles[name])) for name in ALL_LEGS
            }
        fraction = min(1.0, self.phase_time / p.extend_duration)
        height_target = self._extend_start_height + fraction * (
            p.extend_height - self._extend_start_height
        )
        hip_torques = {}
        rotation = np.asarray(state.body_to_world, dtype=float)
        world_up = np.array([0.0, 0.0, 1.0])
        for name in ALL_LEGS:
            q = self._clamped_angle(float(state.leg_angles[name]))
            q_start = self._extend_start_angles[name]
            q_target = q_start + fraction * (self.geometry.q_nominal - q_start)
            jacobian_xz = self.geometry.wheel_jacobian(q)
            jacobian_leg = np.array([jacobian_xz[0], 0.0, jacobian_xz[1]])
            jacobian_world = rotation @ self.mounts[name].rotation_body_from_leg @ jacobian_leg
            support_torque = -(self.mass * self.gravity / len(ALL_LEGS)) * float(
                world_up @ jacobian_world
            )
            torque = p.air_kp * (q_target - q) - p.air_kd * float(leg_rates[name]) + support_torque
            hip_torques[name] = float(np.clip(torque, -p.maximum_hip_torque, p.maximum_hip_torque))
        wheel_torques = self._wheel_torques((0.0, 0.0), wheel_velocities, dt)
        return CrossingCommand(
            phase=self.phase,
            hip_torques=hip_torques,
            wheel_torques=wheel_torques,
            leg_forces={name: 0.0 for name in ALL_LEGS},
            desired_height=height_target,
            desired_pitch=0.0,
            done=False,
            failed=self.failed,
            failure_reason=self.failure_reason,
        )

    def _servo_torque(self, q: float, q_target: float, q_rate: float) -> float:
        """位置伺服力矩：刚度保持 + 阻尼，限幅到髋力矩上限。"""

        p = self.parameters
        torque = p.air_kp * (q_target - q) - p.air_kd * float(q_rate)
        return float(np.clip(torque, -p.maximum_hip_torque, p.maximum_hip_torque))

    def _wheel_torques(
        self,
        speed: tuple[float, float],
        wheel_velocities: Mapping[str, float],
        dt: float,
    ) -> dict[str, float]:
        """把 (前轮速度, 后轮速度) 映射为各轮力矩。"""

        mounts_y = {name: float(self.mounts[name].position_body[1]) for name in ALL_LEGS}
        front_speed, rear_speed = speed
        targets = {}
        for group, group_speed in ((FRONT_LEGS, front_speed), (REAR_LEGS, rear_speed)):
            group_targets = wheel_speed_targets(
                TeleopCommand(v_x=group_speed, omega_yaw=0.0),
                self.parameters.wheel_radius,
                {name: mounts_y[name] for name in group},
            )
            targets.update(group_targets)
        return self.wheel_controller.update(targets, wheel_velocities, dt=dt)

    def _clamped_angle(self, q: float) -> float:
        """Clamp a measured hip angle into the analytical working interval."""

        return float(np.clip(q, self.geometry.q_min, self.geometry.q_max))

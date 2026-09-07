"""MuJoCo integration layer for the four-leg virtual-model controller."""

from __future__ import annotations

from dataclasses import dataclass
from math import asin, atan2

import numpy as np
from numpy.typing import NDArray

from ascento_dog.control import (
    AttitudeGains,
    HeightPID,
    PIDGains,
    QuadrupedVMC,
    TeleopCommand,
    VMCCommand,
    VMCState,
    WheelVelocityController,
    wheel_speed_targets,
)
from ascento_dog.control import (
    LegMount as ControlLegMount,
)
from ascento_dog.kinematics import DEFAULT_GEOMETRY
from ascento_dog.simulation.mujoco_quadruped import LEG_MOUNTS, WHEEL_RADIUS


@dataclass(frozen=True)
class DefaultVMCParameters:
    """Illustrative gains and limits used by the MuJoCo demonstration."""

    height_kp: float = 1000.0
    height_ki: float = 200.0
    height_kd: float = 260.0
    height_integral_limit: float = 0.15
    height_thrust_limit: float = 180.0
    roll_kp: float = 180.0
    roll_kd: float = 28.0
    pitch_kp: float = 480.0
    pitch_kd: float = 100.0
    maximum_leg_force: float = 120.0
    maximum_hip_torque: float = 40.0


_DEFAULT_VMC_PARAMETERS = DefaultVMCParameters()


def create_default_vmc(
    model, parameters: DefaultVMCParameters = _DEFAULT_VMC_PARAMETERS
) -> QuadrupedVMC:
    """Construct the demonstration controller from the compiled model mass.

    The total mass is read from MuJoCo instead of being duplicated in control
    code.  All gains and force/torque limits remain explicit illustrative
    values until CAD and actuator identification are available.
    """

    mounts: dict[str, ControlLegMount] = {}
    for name, mount in LEG_MOUNTS.items():
        mounts[name] = ControlLegMount(mount.position, mount.rotation)
    return QuadrupedVMC(
        mass=float(np.sum(model.body_mass)),
        mounts=mounts,
        height_pid=HeightPID(
            PIDGains(
                parameters.height_kp,
                parameters.height_ki,
                parameters.height_kd,
                integral_limit=parameters.height_integral_limit,
                output_limit=parameters.height_thrust_limit,
            )
        ),
        attitude_gains=AttitudeGains(
            parameters.roll_kp,
            parameters.roll_kd,
            parameters.pitch_kp,
            parameters.pitch_kd,
        ),
        maximum_leg_force=parameters.maximum_leg_force,
        maximum_hip_torque=parameters.maximum_hip_torque,
    )


def read_vmc_state(model, data) -> VMCState:
    """Read the chassis and analytical leg state from MuJoCo."""

    import mujoco

    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    rotation = data.xmat[chassis_id].reshape(3, 3).copy()
    yaw, pitch, roll = yaw_pitch_roll_from_rotation(rotation)
    local_velocity = np.zeros(6)
    mujoco.mj_objectVelocity(
        model,
        data,
        mujoco.mjtObj.mjOBJ_BODY,
        chassis_id,
        local_velocity,
        1,
    )
    leg_angles: dict[str, float] = {}
    for name in LEG_MOUNTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{name}_hip_drive")
        leg_angles[name] = float(
            DEFAULT_GEOMETRY.q_nominal + data.qpos[model.jnt_qposadr[joint_id]]
        )
    return VMCState(
        height=float(data.xpos[chassis_id, 2]),
        vertical_velocity=float(data.qvel[2]),
        roll=roll,
        pitch=pitch,
        angular_velocity_body=local_velocity[:3].copy(),
        body_to_world=rotation,
        leg_angles=leg_angles,
        yaw=yaw,
        center_of_mass_body=(
            rotation.T @ (data.subtree_com[chassis_id] - data.xpos[chassis_id])
        ).copy(),
    )


def apply_vmc_command(model, data, command: VMCCommand) -> None:
    """Write hip torques to MuJoCo and leave wheel motors unpowered."""

    import mujoco

    data.ctrl[:] = 0.0
    for name, torque in command.hip_torques.items():
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_hip_motor")
        if actuator_id < 0:
            raise KeyError(f"hip actuator for {name!r} was not found")
        data.ctrl[actuator_id] = torque


def step_vmc(
    model,
    data,
    controller: QuadrupedVMC,
    *,
    desired_height: float,
) -> VMCCommand:
    """Compute and apply one VMC command, then advance MuJoCo one step."""

    import mujoco

    state = read_vmc_state(model, data)
    command = controller.compute(state, desired_height=desired_height, dt=float(model.opt.timestep))
    apply_vmc_command(model, data, command)
    mujoco.mj_step(model, data)
    return command


def apply_body_disturbance(
    model,
    data,
    *,
    force_world: tuple[float, float, float] = (0.0, 0.0, 0.0),
    torque_world: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    """Apply a world-frame disturbance wrench at the chassis center."""

    import mujoco

    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    data.xfrc_applied[chassis_id, :3] = np.asarray(force_world, dtype=float)
    data.xfrc_applied[chassis_id, 3:] = np.asarray(torque_world, dtype=float)


def yaw_pitch_roll_from_rotation(rotation: np.ndarray) -> tuple[float, float, float]:
    """Return ZYX yaw, pitch, and roll from a body-to-world rotation matrix."""

    pitch = asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    yaw = atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    return yaw, pitch, roll


def read_wheel_velocities(model, data) -> dict[str, float]:
    """Read each wheel's spin angular velocity in rad/s."""

    import mujoco

    velocities: dict[str, float] = {}
    for name in LEG_MOUNTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{name}_wheel_spin")
        if joint_id < 0:
            raise KeyError(f"wheel spin joint for {name!r} not found")
        velocities[name] = float(data.qvel[model.jnt_dofadr[joint_id]])
    return velocities


def apply_wheel_command(model, data, wheel_torques: dict[str, float]) -> None:
    """Write per-wheel motor torques to MuJoCo, leaving hips untouched."""

    import mujoco

    for name, torque in wheel_torques.items():
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_wheel_motor")
        if actuator_id < 0:
            raise KeyError(f"wheel actuator for {name!r} not found")
        data.ctrl[actuator_id] = torque


def step_teleop(
    model,
    data,
    controller: QuadrupedVMC,
    wheel_controller: WheelVelocityController,
    command: TeleopCommand,
    *,
    desired_height: float,
) -> VMCCommand:
    """Compute and apply one VMC + teleop command, then step MuJoCo."""

    import mujoco

    state = read_vmc_state(model, data)
    hip_command = controller.compute(
        state, desired_height=desired_height, dt=float(model.opt.timestep)
    )
    mounts_y = {name: mount.position[1] for name, mount in LEG_MOUNTS.items()}
    targets = wheel_speed_targets(command, WHEEL_RADIUS, mounts_y)
    measured = read_wheel_velocities(model, data)
    wheel_torques = wheel_controller.update(targets, measured, dt=float(model.opt.timestep))
    apply_vmc_command(model, data, hip_command)
    apply_wheel_command(model, data, wheel_torques)
    mujoco.mj_step(model, data)
    return hip_command


def read_imu_attitude(model, data) -> tuple[float, float, float]:
    """Return ZYX ``(yaw, pitch, roll)`` in radians from the chassis IMU.

    Reads the model's ``imu_quat`` ``framequat`` sensor on the chassis body and
    converts the quaternion to a rotation matrix, then to ZYX Euler angles
    (same convention as ``yaw_pitch_roll_from_rotation``).
    """

    import mujoco

    quat = np.asarray(data.sensor("imu_quat").data, dtype=float)
    if quat.shape != (4,):
        raise ValueError("imu_quat sensor must return a 4-vector")
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, quat)
    return yaw_pitch_roll_from_rotation(rotation.reshape(3, 3))


def read_wheel_centers(model, data) -> dict[str, np.ndarray]:
    """返回每个轮心的世界系位置 (x, y, z)，单位 m。"""

    import mujoco

    centers: dict[str, NDArray[np.float64]] = {}
    for name in LEG_MOUNTS:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{name}_wheel")
        if body_id < 0:
            raise KeyError(f"wheel body for {name!r} not found")
        centers[name] = np.array(data.xpos[body_id], dtype=float)
    return centers


def read_hip_rates(model, data) -> dict[str, float]:
    """返回四个髋关节角速度，单位 rad/s。"""

    import mujoco

    rates: dict[str, float] = {}
    for name in LEG_MOUNTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{name}_hip_drive")
        if joint_id < 0:
            raise KeyError(f"hip joint for {name!r} not found")
        rates[name] = float(data.qvel[model.jnt_dofadr[joint_id]])
    return rates


def apply_crossing_command(model, data, command) -> None:
    """写入跨越台阶指令的髋关节与车轮力矩。

    ``command`` 为 :class:`~ascento_dog.control.step_crossing.CrossingCommand`，
    髋与轮力矩单位均为 N·m，其余执行器保持为零。
    """

    import mujoco

    data.ctrl[:] = 0.0
    for name, torque in command.hip_torques.items():
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_hip_motor")
        if actuator_id < 0:
            raise KeyError(f"hip actuator for {name!r} not found")
        data.ctrl[actuator_id] = torque
    for name, torque in command.wheel_torques.items():
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_wheel_motor")
        if actuator_id < 0:
            raise KeyError(f"wheel actuator for {name!r} not found")
        data.ctrl[actuator_id] = torque

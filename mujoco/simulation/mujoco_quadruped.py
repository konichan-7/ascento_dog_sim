"""MuJoCo adapter for the four-legged, four-wheeled robot model."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from ascento_dog.kinematics import DEFAULT_GEOMETRY, FourBarGeometry, LegPose

MODEL_PATH = Path(__file__).resolve().parents[2] / "mujoco" / "quadruped.xml"
STEP_MODEL_PATH = Path(__file__).resolve().parents[2] / "mujoco" / "quadruped_step.xml"
WHEEL_RADIUS = 0.065
CHASSIS_SIZE = np.array([0.60, 0.36, 0.15], dtype=float)


# 腿安装座绕 z 轴旋转 180°：解析腿系 +x 指向机体 -x（屈膝朝向机体后方）。
LEG_ROTATION = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]], dtype=float)


@dataclass(frozen=True)
class LegMount:
    """One linkage frame expressed in the chassis frame, in meters.

    ``position`` is the hip point A in the chassis frame;
    ``rotation`` maps leg-frame vectors into the chassis frame.
    """

    position: NDArray[np.float64]
    rotation: NDArray[np.float64] = field(default_factory=lambda: np.eye(3))


LEG_MOUNTS = {
    "front_left": LegMount(np.array([0.24, 0.20, 0.0]), LEG_ROTATION),
    "front_right": LegMount(np.array([0.24, -0.20, 0.0]), LEG_ROTATION),
    "rear_left": LegMount(np.array([-0.24, 0.20, 0.0]), LEG_ROTATION),
    "rear_right": LegMount(np.array([-0.24, -0.20, 0.0]), LEG_ROTATION),
}


def load_quadruped_model():
    """Load and return ``(mjModel, mjData)`` for the complete robot."""

    import mujoco

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    return model, data


def set_quadruped_pose(
    model,
    data,
    q: float,
    geometry: FourBarGeometry = DEFAULT_GEOMETRY,
    *,
    wheels_on_floor: bool = True,
) -> dict[str, LegPose]:
    """Set all four linkages to ``q`` and optionally place the wheels on ``z=0``.

    ``q`` is the local hip angle for every leg in radians.  All four MJCF mount
    frames use the same analytical +x direction, branch, and joint mapping.
    """

    import mujoco

    pose = geometry.forward(q)
    nominal = geometry.forward(geometry.q_nominal)
    mujoco.mj_resetData(model, data)

    if wheels_on_floor:
        free_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base")
        free_qpos_address = model.jnt_qposadr[free_id]
        data.qpos[free_qpos_address + 2] = WHEEL_RADIUS - pose.e[1]

    for leg_name in LEG_MOUNTS:
        _set_joint_qpos(model, data, f"{leg_name}_hip_drive", q - geometry.q_nominal)
        _set_joint_qpos(
            model,
            data,
            f"{leg_name}_inner_passive",
            (pose.psi - q) - (nominal.psi - geometry.q_nominal),
        )
        _set_joint_qpos(
            model,
            data,
            f"{leg_name}_pin_passive",
            pose.phi - nominal.phi,
        )

    mujoco.mj_forward(model, data)
    return {leg_name: pose for leg_name in LEG_MOUNTS}


def model_leg_points_in_chassis(model, data, leg_name: str) -> dict[str, NDArray[np.float64]]:
    """Read one leg's A-E sites in the chassis frame, in meters."""

    import mujoco

    if leg_name not in LEG_MOUNTS:
        raise KeyError(f"unknown leg {leg_name!r}")
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    chassis_position = data.xpos[chassis_id]
    chassis_rotation = data.xmat[chassis_id].reshape(3, 3)
    points: dict[str, NDArray[np.float64]] = {}
    for point_name in "ABCDE":
        site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, f"{leg_name}_point_{point_name}"
        )
        points[point_name] = (
            chassis_rotation.T @ (data.site_xpos[site_id] - chassis_position)
        ).copy()
    return points


def expected_leg_points_in_chassis(leg_name: str, pose: LegPose) -> dict[str, NDArray[np.float64]]:
    """Map analytical A-E points into the chassis frame for one mount."""

    mount = LEG_MOUNTS[leg_name]
    result: dict[str, NDArray[np.float64]] = {}
    for point_name, point_xz in pose.points().items():
        local = np.array([point_xz[0], 0.0, point_xz[1]])
        result[point_name] = mount.position + mount.rotation @ local
    return result


def leg_loop_error(model, data, leg_name: str) -> float:
    """Return one leg's distance between its two C closure sites, in meters."""

    import mujoco

    lower = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{leg_name}_point_C_lower")
    upper = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"{leg_name}_point_C_upper")
    return float(np.linalg.norm(data.site_xpos[lower] - data.site_xpos[upper]))


def wheel_bottom_heights(model, data) -> dict[str, float]:
    """Return each wheel's lowest z coordinate, in meters."""

    import mujoco

    heights: dict[str, float] = {}
    for leg_name in LEG_MOUNTS:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{leg_name}_wheel")
        heights[leg_name] = float(data.xpos[body_id, 2] - WHEEL_RADIUS)
    return heights


def _set_joint_qpos(model, data, name: str, value: float) -> None:
    import mujoco

    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise KeyError(f"joint {name!r} not found")
    data.qpos[model.jnt_qposadr[joint_id]] = value

"""MuJoCo adapter and validation helpers for the floating single-leg MJCF."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from ascento_dog.kinematics import DEFAULT_GEOMETRY, FourBarGeometry, LegPose

MODEL_PATH = Path(__file__).resolve().parents[2] / "mujoco" / "single_leg.xml"


@dataclass(frozen=True)
class MujocoVerification:
    """Maximum errors measured over a MuJoCo kinematic sweep."""

    samples: int
    max_point_error_m: float
    max_loop_error_m: float


def load_single_leg_model():
    """Load and return ``(mjModel, mjData)`` for the single floating leg."""

    import mujoco

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    return model, data


def set_kinematic_pose(
    model,
    data,
    q: float,
    geometry: FourBarGeometry = DEFAULT_GEOMETRY,
) -> LegPose:
    """Set all hinge coordinates to the analytical closed pose for absolute ``q``."""

    import mujoco

    pose = geometry.forward(q)
    nominal = geometry.forward(geometry.q_nominal)
    mujoco.mj_resetData(model, data)

    _set_joint_qpos(model, data, "hip_drive", q - geometry.q_nominal)
    _set_joint_qpos(
        model,
        data,
        "inner_passive",
        (pose.psi - q) - (nominal.psi - geometry.q_nominal),
    )
    _set_joint_qpos(model, data, "pin_passive", pose.phi - nominal.phi)
    mujoco.mj_forward(model, data)
    return pose


def model_points_in_base(model, data) -> dict[str, NDArray[np.float64]]:
    """Read MuJoCo sites A-E in the base-fixed analytical ``(x, z)`` frame."""

    import mujoco

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    base_position = data.xpos[base_id]
    base_rotation = data.xmat[base_id].reshape(3, 3)

    names = {
        "A": "point_A",
        "B": "point_B",
        "C": "point_C_lower",
        "D": "point_D",
        "E": "point_E",
    }
    points: dict[str, NDArray[np.float64]] = {}
    for point_name, site_name in names.items():
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        relative = base_rotation.T @ (data.site_xpos[site_id] - base_position)
        points[point_name] = relative[[0, 2]].copy()
    return points


def loop_error(model, data) -> float:
    """Return distance in meters between the lower-link and upper-link C sites."""

    import mujoco

    lower = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "point_C_lower")
    upper = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "point_C_upper")
    return float(np.linalg.norm(data.site_xpos[lower] - data.site_xpos[upper]))


def verify_mujoco_kinematics(
    *,
    samples: int = 51,
    geometry: FourBarGeometry = DEFAULT_GEOMETRY,
) -> MujocoVerification:
    """Compare MuJoCo sites with analytical A-E points across the working range."""

    if samples < 2:
        raise ValueError("samples must be at least 2")
    model, data = load_single_leg_model()
    max_point_error = 0.0
    max_loop_error = 0.0
    for q in np.linspace(geometry.q_min, geometry.q_max, samples):
        pose = set_kinematic_pose(model, data, float(q), geometry)
        actual = model_points_in_base(model, data)
        for point_name, expected in pose.points().items():
            max_point_error = max(
                max_point_error, float(np.linalg.norm(actual[point_name] - expected))
            )
        max_loop_error = max(max_loop_error, loop_error(model, data))
    return MujocoVerification(
        samples=samples,
        max_point_error_m=max_point_error,
        max_loop_error_m=max_loop_error,
    )


def _set_joint_qpos(model, data, name: str, value: float) -> None:
    import mujoco

    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise KeyError(f"joint {name!r} not found")
    data.qpos[model.jnt_qposadr[joint_id]] = value

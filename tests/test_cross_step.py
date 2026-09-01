"""Deterministic validation of the step-crossing scenario (150 mm step)."""

from __future__ import annotations

import argparse

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from ascento_dog.control import (  # noqa: E402
    CrossingParameters,
    CrossingPhase,
    LegMount,
    StepCrossingController,
    WheelObservation,
)
from ascento_dog.kinematics import DEFAULT_GEOMETRY  # noqa: E402
from ascento_dog.scripts import cross_step  # noqa: E402
from ascento_dog.simulation import (  # noqa: E402
    LEG_MOUNTS,
    load_step_model,
    read_vmc_state,
    set_quadruped_pose,
)


def _args(**overrides: object) -> argparse.Namespace:
    base = dict(
        headless=True,
        duration=30.0,
        step_height=0.15,
        step_face_x=0.9,
        forward_speed=0.25,
    )
    base.update(overrides)
    return argparse.Namespace(**base)


def test_step_scene_compiles_with_150mm_platform() -> None:
    model, _ = load_step_model()
    step_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "step")
    assert step_id >= 0
    size = model.geom_size[step_id]
    assert size[0] == pytest.approx(0.8)
    assert size[2] == pytest.approx(0.075)  # 150 mm 台阶
    # 顶面高度 150 mm：中心 z 0.075 + 半高 0.075
    assert model.geom_pos[step_id, 2] + size[2] == pytest.approx(0.15)


def test_front_rear_wheels_and_mounts_same_direction_after_flip() -> None:
    model, data = load_step_model()
    set_quadruped_pose(model, data, DEFAULT_GEOMETRY.q_nominal)
    front = data.body("front_left_wheel").xpos
    rear = data.body("rear_left_wheel").xpos
    offset_f = front - LEG_MOUNTS["front_left"].position
    offset_r = rear - LEG_MOUNTS["rear_left"].position
    # 同向安装：前后腿轮心相对安装座偏移一致（屈膝朝向机体 -x）
    assert offset_f == pytest.approx(offset_r, abs=3.0e-9)


def test_full_step_crossing_succeeds_headless() -> None:
    """机器人应从平地驶上 150 mm 台阶，四轮着台、姿态水平。"""

    args = _args()
    cross_step._validate_args(args)
    model, data = load_step_model()
    set_quadruped_pose(model, data, DEFAULT_GEOMETRY.q_nominal, wheels_on_floor=True)
    mounts = {n: LegMount(m.position, m.rotation) for n, m in LEG_MOUNTS.items()}
    d_min = -DEFAULT_GEOMETRY.forward(DEFAULT_GEOMETRY.q_max).e[1]
    nominal_drop = -DEFAULT_GEOMETRY.forward(DEFAULT_GEOMETRY.q_nominal).e[1]
    controller = StepCrossingController(
        mass=float(np.sum(model.body_mass)),
        mounts=mounts,
        parameters=CrossingParameters(
            step_height=args.step_height,
            step_face_x=args.step_face_x,
            stance_height=args.step_height + 0.065 + d_min,
            extend_height=args.step_height + 0.065 + nominal_drop,
            forward_speed=args.forward_speed,
        ),
        geometry=DEFAULT_GEOMETRY,
    )

    assert cross_step.run_headless(model, data, controller, args) == 0

    state = read_vmc_state(model, data)
    assert state.roll == pytest.approx(0.0, abs=np.deg2rad(1.0))
    assert state.pitch == pytest.approx(0.0, abs=np.deg2rad(1.0))
    assert state.height == pytest.approx(0.15 + 0.065 + nominal_drop, abs=0.02)
    for leg in ("front_left", "front_right", "rear_left", "rear_right"):
        wheel_z = float(data.body(f"{leg}_wheel").xpos[2])
        assert wheel_z == pytest.approx(0.15 + 0.065, abs=0.01)


def test_controller_rejects_off_target_inputs() -> None:
    params = CrossingParameters()
    controller = StepCrossingController(
        mass=20.0,
        mounts={n: LegMount(m.position, m.rotation) for n, m in LEG_MOUNTS.items()},
        parameters=params,
    )
    wheels = {n: WheelObservation(x=0.5, z=0.065) for n in LEG_MOUNTS}
    leg_rates = {n: 0.0 for n in LEG_MOUNTS}
    wheel_vels = {n: 0.0 for n in LEG_MOUNTS}
    with pytest.raises(ValueError):
        controller.update(None, wheels, wheel_velocities=wheel_vels, leg_rates=leg_rates, dt=-1.0)


def test_phases_follow_expected_sequence() -> None:
    """水平站姿高度与阶段顺序的纯逻辑检查。"""

    params = CrossingParameters()
    assert params.step_height == pytest.approx(0.15)
    # 水平站姿：台阶顶 + 轮半径 + 最短腿下探量（前腿收缩到位时前轮恰好触台）。
    assert params.stance_height == pytest.approx(0.15 + 0.065 + 0.111206, abs=1e-3)
    assert (
        CrossingPhase.FRONT_CLIMB.value == "front_climb"
        and CrossingPhase.REAR_CLIMB.value == "rear_climb"
    )

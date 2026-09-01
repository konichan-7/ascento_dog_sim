import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from ascento_dog.kinematics import DEFAULT_GEOMETRY
from ascento_dog.simulation.mujoco_leg import (
    load_single_leg_model,
    loop_error,
    model_points_in_base,
    set_kinematic_pose,
    verify_mujoco_kinematics,
)


def test_mjcf_compiles_as_a_free_closed_loop() -> None:
    model, _ = load_single_leg_model()
    assert model.nq == 10  # free root (7) plus three planar hinges
    assert model.neq == 1
    assert model.nu == 1


@pytest.mark.parametrize(
    "q",
    [DEFAULT_GEOMETRY.q_min, DEFAULT_GEOMETRY.q_nominal, DEFAULT_GEOMETRY.q_max],
)
def test_mujoco_sites_match_analytical_points(q: float) -> None:
    model, data = load_single_leg_model()
    pose = set_kinematic_pose(model, data, q)
    actual = model_points_in_base(model, data)
    for point_name, expected in pose.points().items():
        assert actual[point_name] == pytest.approx(expected, abs=2.0e-9)
    assert loop_error(model, data) < 2.0e-9


def test_full_mujoco_sweep() -> None:
    report = verify_mujoco_kinematics(samples=51)
    assert report.max_point_error_m < 2.0e-9
    assert report.max_loop_error_m < 2.0e-9


def test_actuated_floating_loop_remains_finite_and_converges() -> None:
    model, data = load_single_leg_model()
    set_kinematic_pose(model, data, DEFAULT_GEOMETRY.q_nominal)
    hip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hip_drive")
    hip_qpos_address = model.jnt_qposadr[hip_id]
    target = np.deg2rad(15.0)
    data.ctrl[0] = target

    maximum_loop_error = 0.0
    for _ in range(1000):
        mujoco.mj_step(model, data)
        maximum_loop_error = max(maximum_loop_error, loop_error(model, data))

    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))
    assert data.qpos[hip_qpos_address] == pytest.approx(target, abs=2.0e-5)
    assert maximum_loop_error < 1.0e-3
    assert loop_error(model, data) < 1.0e-6

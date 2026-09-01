import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from ascento_dog.kinematics import DEFAULT_GEOMETRY  # noqa: E402
from ascento_dog.simulation.mujoco_quadruped import (  # noqa: E402
    CHASSIS_SIZE,
    LEG_MOUNTS,
    expected_leg_points_in_chassis,
    leg_loop_error,
    load_quadruped_model,
    model_leg_points_in_chassis,
    set_quadruped_pose,
    wheel_bottom_heights,
)
from ascento_dog.simulation.mujoco_vmc import (  # noqa: E402
    apply_body_disturbance,
    create_default_vmc,
    read_vmc_state,
    step_vmc,
)


def test_quadruped_compiles_with_four_closed_loops_and_eight_actuators() -> None:
    model, _ = load_quadruped_model()
    assert model.nq == 23  # free root (7) plus four legs x four hinge coordinates
    assert model.nv == 22
    assert model.neq == 4
    assert model.nu == 8
    assert tuple(CHASSIS_SIZE) == (0.60, 0.36, 0.15)
    assert model.opt.gravity == pytest.approx([0.0, 0.0, -9.81])


def test_quadruped_has_four_independent_wheel_joints() -> None:
    model, _ = load_quadruped_model()
    for leg_name in LEG_MOUNTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{leg_name}_wheel_spin")
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{leg_name}_wheel_motor"
        )
        assert joint_id >= 0
        assert actuator_id >= 0
        assert model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_HINGE


def test_wheels_have_ground_contact_and_hips_have_torque_motors() -> None:
    model, _ = load_quadruped_model()
    for leg_name in LEG_MOUNTS:
        wheel_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg_name}_wheel_geom")
        hip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{leg_name}_hip_motor")
        assert model.geom_contype[wheel_id] == 1
        assert model.geom_conaffinity[wheel_id] == 1
        assert hip_id >= 0
        assert model.actuator_ctrlrange[hip_id] == pytest.approx([-40.0, 40.0])


@pytest.mark.parametrize(
    "q",
    [DEFAULT_GEOMETRY.q_min, DEFAULT_GEOMETRY.q_nominal, DEFAULT_GEOMETRY.q_max],
)
def test_all_leg_sites_match_analytical_kinematics(q: float) -> None:
    model, data = load_quadruped_model()
    poses = set_quadruped_pose(model, data, q)
    for leg_name, pose in poses.items():
        actual = model_leg_points_in_chassis(model, data, leg_name)
        expected = expected_leg_points_in_chassis(leg_name, pose)
        for point_name in "ABCDE":
            assert actual[point_name] == pytest.approx(expected[point_name], abs=3.0e-9)
        assert leg_loop_error(model, data, leg_name) < 3.0e-9


@pytest.mark.parametrize(
    "q",
    [DEFAULT_GEOMETRY.q_min, DEFAULT_GEOMETRY.q_nominal, DEFAULT_GEOMETRY.q_max],
)
def test_synchronous_pose_places_all_wheels_on_floor(q: float) -> None:
    model, data = load_quadruped_model()
    set_quadruped_pose(model, data, q, wheels_on_floor=True)
    heights = np.array(list(wheel_bottom_heights(model, data).values()))
    assert heights == pytest.approx(np.zeros(4), abs=3.0e-9)


def test_all_four_linkage_frames_point_in_the_same_direction() -> None:
    model, data = load_quadruped_model()
    set_quadruped_pose(model, data, DEFAULT_GEOMETRY.q_nominal)
    front = model_leg_points_in_chassis(model, data, "front_left")["E"]
    rear = model_leg_points_in_chassis(model, data, "rear_left")["E"]
    front_offset = front - LEG_MOUNTS["front_left"].position
    rear_offset = rear - LEG_MOUNTS["rear_left"].position
    assert front_offset == pytest.approx(rear_offset, abs=3.0e-9)


def test_vmc_tracks_height_and_recovers_from_attitude_disturbance() -> None:
    model, data = load_quadruped_model()
    set_quadruped_pose(model, data, DEFAULT_GEOMETRY.q_nominal, wheels_on_floor=True)
    controller = create_default_vmc(model)
    maximum_height_error = 0.0
    maximum_roll = 0.0
    maximum_pitch = 0.0

    while data.time < 8.0:
        desired_height = 0.37 if data.time < 2.0 else 0.41
        if 4.0 <= data.time < 4.12:
            apply_body_disturbance(model, data, torque_world=(30.0, -20.0, 0.0))
        else:
            apply_body_disturbance(model, data)
        step_vmc(model, data, controller, desired_height=desired_height)
        state = read_vmc_state(model, data)
        maximum_height_error = max(maximum_height_error, abs(state.height - desired_height))
        maximum_roll = max(maximum_roll, abs(state.roll))
        maximum_pitch = max(maximum_pitch, abs(state.pitch))
        assert np.all(np.isfinite(data.qpos))
        assert np.all(np.isfinite(data.qvel))

    final = read_vmc_state(model, data)
    assert maximum_height_error < 0.035
    assert maximum_roll < np.deg2rad(4.0)
    assert maximum_pitch < np.deg2rad(3.0)
    assert final.height == pytest.approx(0.41, abs=0.025)
    assert final.roll == pytest.approx(0.0, abs=np.deg2rad(0.5))
    assert final.pitch == pytest.approx(0.0, abs=np.deg2rad(0.5))

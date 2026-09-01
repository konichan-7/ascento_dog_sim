import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from ascento_dog.control import DriveCommand, WheelSpeedGains, WheelVelocityController  # noqa: E402
from ascento_dog.kinematics import DEFAULT_GEOMETRY  # noqa: E402
from ascento_dog.simulation import (  # noqa: E402
    create_default_vmc,
    load_quadruped_model,
    read_imu_attitude,
    read_vmc_state,
    set_quadruped_pose,
    step_drive,
)


def _chassis_x(model, data) -> float:
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    return float(data.xpos[chassis_id, 0])


def test_forward_command_translates_the_robot_forward() -> None:
    model, data = load_quadruped_model()
    set_quadruped_pose(model, data, DEFAULT_GEOMETRY.q_nominal, wheels_on_floor=True)
    controller = create_default_vmc(model)
    wheel_controller = WheelVelocityController(
        WheelSpeedGains(kp=0.5, ki=0.2, integral_limit=3.0, output_limit=10.0)
    )
    start_x = _chassis_x(model, data)
    command = DriveCommand(v_x=0.5, omega_yaw=0.0)

    while data.time < 3.0:
        step_drive(model, data, controller, wheel_controller, command, desired_height=0.37)
        assert np.all(np.isfinite(data.qpos))
        assert np.all(np.isfinite(data.qvel))

    traveled = _chassis_x(model, data) - start_x
    assert traveled > 0.2
    state = read_vmc_state(model, data)
    assert abs(state.roll) < np.deg2rad(5.0)
    assert abs(state.pitch) < np.deg2rad(5.0)


def test_yaw_command_turns_the_robot_left() -> None:
    model, data = load_quadruped_model()
    set_quadruped_pose(model, data, DEFAULT_GEOMETRY.q_nominal, wheels_on_floor=True)
    controller = create_default_vmc(model)
    wheel_controller = WheelVelocityController(
        WheelSpeedGains(kp=0.5, ki=0.2, integral_limit=3.0, output_limit=10.0)
    )
    start_yaw = read_vmc_state(model, data).yaw
    command = DriveCommand(v_x=0.0, omega_yaw=1.0)

    while data.time < 1.5:
        step_drive(model, data, controller, wheel_controller, command, desired_height=0.37)
        assert np.all(np.isfinite(data.qpos))
        assert np.all(np.isfinite(data.qvel))

    # positive omega_yaw (the A key) must yaw counterclockwise from above (left turn)
    assert read_vmc_state(model, data).yaw - start_yaw > np.deg2rad(3.0)


def test_imu_sensor_reports_level_attitude_at_rest() -> None:
    model, data = load_quadruped_model()
    set_quadruped_pose(model, data, DEFAULT_GEOMETRY.q_nominal, wheels_on_floor=True)
    for sensor_name in ("imu_quat", "imu_gyro", "imu_accel"):
        assert data.sensor(sensor_name) is not None
    yaw, pitch, roll = read_imu_attitude(model, data)
    assert np.all(np.isfinite((yaw, pitch, roll)))
    assert roll == pytest.approx(0.0, abs=1.0e-9)
    assert pitch == pytest.approx(0.0, abs=1.0e-9)

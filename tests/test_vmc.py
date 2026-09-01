from math import cos, radians, sin

import numpy as np
import pytest

from ascento_dog.control import (
    AttitudeGains,
    HeightPID,
    LegMount,
    PIDGains,
    QuadrupedVMC,
    VMCState,
    allocate_vertical_forces,
)
from ascento_dog.kinematics import DEFAULT_GEOMETRY
from ascento_dog.simulation import yaw_pitch_roll_from_rotation


LEG_NAMES = ("front_left", "front_right", "rear_left", "rear_right")


def _mounts() -> dict[str, LegMount]:
    return {
        "front_left": LegMount(np.array([0.24, 0.20, 0.0])),
        "front_right": LegMount(np.array([0.24, -0.20, 0.0])),
        "rear_left": LegMount(np.array([-0.24, 0.20, 0.0])),
        "rear_right": LegMount(np.array([-0.24, -0.20, 0.0])),
    }


def _rotation(roll: float, pitch: float) -> np.ndarray:
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    roll_matrix = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    pitch_matrix = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return pitch_matrix @ roll_matrix


def test_yaw_pitch_roll_extraction() -> None:
    yaw, pitch, roll = map(radians, (20.0, -10.0, 15.0))
    cy, sy = cos(yaw), sin(yaw)
    yaw_matrix = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    rotation = yaw_matrix @ _rotation(roll, pitch)
    recovered = yaw_pitch_roll_from_rotation(rotation)
    assert recovered == pytest.approx([yaw, pitch, roll], abs=1.0e-12)


def test_level_symmetric_force_allocation_has_closed_form_solution() -> None:
    a, b, height = 0.24, 0.20, 0.30
    points = np.array(
        [[a, b, -height], [a, -b, -height], [-a, b, -height], [-a, -b, -height]]
    )
    desired = np.array([200.0, 8.0, -12.0])
    forces, achieved = allocate_vertical_forces(desired, points, np.eye(3))
    expected = np.array(
        [
            50.0 + 8.0 / (4.0 * b) - (-12.0) / (4.0 * a),
            50.0 - 8.0 / (4.0 * b) - (-12.0) / (4.0 * a),
            50.0 + 8.0 / (4.0 * b) + (-12.0) / (4.0 * a),
            50.0 - 8.0 / (4.0 * b) + (-12.0) / (4.0 * a),
        ]
    )
    assert forces == pytest.approx(expected, abs=1.0e-10)
    assert achieved == pytest.approx(desired, abs=1.0e-10)


def test_pose_dependent_force_allocation_handles_roll_and_pitch() -> None:
    points = np.array(
        [[0.24, 0.20, -0.30], [0.24, -0.20, -0.30], [-0.24, 0.20, -0.30], [-0.24, -0.20, -0.30]]
    )
    rotation = _rotation(0.12, -0.08)
    desired = np.array([200.0, -4.0, 6.0])
    forces, achieved = allocate_vertical_forces(
        desired, points, rotation, minimum_force=0.0, maximum_force=100.0
    )
    assert np.all((0.0 <= forces) & (forces <= 100.0))
    assert achieved == pytest.approx(desired, abs=1.0e-10)


def test_force_limits_are_respected_for_an_infeasible_wrench() -> None:
    points = np.array(
        [[0.24, 0.20, -0.30], [0.24, -0.20, -0.30], [-0.24, 0.20, -0.30], [-0.24, -0.20, -0.30]]
    )
    forces, achieved = allocate_vertical_forces(
        np.array([1000.0, 0.0, 0.0]),
        points,
        np.eye(3),
        minimum_force=0.0,
        maximum_force=80.0,
    )
    assert forces == pytest.approx(np.full(4, 80.0))
    assert achieved[0] == pytest.approx(320.0)


def test_height_pid_output_and_anti_windup() -> None:
    pid = HeightPID(PIDGains(100.0, 50.0, 10.0, integral_limit=0.2, output_limit=20.0))
    assert pid.update(0.4, 0.3, 0.1, 0.01) == pytest.approx(9.05)
    for _ in range(100):
        assert pid.update(1.0, 0.0, 0.0, 0.01) == pytest.approx(20.0)
    assert pid.integral == pytest.approx(0.001)
    output = pid.update(0.0, 1.0, 0.0, 0.01)
    assert output == pytest.approx(-20.0)
    assert abs(pid.integral) <= 0.001 + 1.0e-12


def test_pid_nan_limits_rejected() -> None:
    with pytest.raises(ValueError):
        PIDGains(100.0, 50.0, 10.0, integral_limit=float("nan"))


def test_quadruped_vmc_compensates_gravity_and_maps_force_to_torque() -> None:
    mass = 20.0
    controller = QuadrupedVMC(
        mass=mass,
        mounts=_mounts(),
        height_pid=HeightPID(PIDGains(1000.0, 0.0, 100.0, output_limit=300.0)),
        attitude_gains=AttitudeGains(100.0, 10.0, 120.0, 12.0),
        maximum_leg_force=100.0,
        maximum_hip_torque=40.0,
    )
    state = VMCState(
        height=0.37,
        vertical_velocity=0.0,
        roll=0.0,
        pitch=0.0,
        angular_velocity_body=np.zeros(3),
        body_to_world=np.eye(3),
        leg_angles={name: DEFAULT_GEOMETRY.q_nominal for name in LEG_NAMES},
    )
    command = controller.compute(state, desired_height=0.37, dt=0.002)
    forces = np.array(list(command.leg_forces.values()))
    assert np.sum(forces) == pytest.approx(mass * 9.81)
    assert command.achieved_wrench == pytest.approx([mass * 9.81, 0.0, 0.0], abs=1.0e-10)
    jacobian_z = DEFAULT_GEOMETRY.wheel_jacobian(DEFAULT_GEOMETRY.q_nominal)[1]
    assert list(command.hip_torques.values()) == pytest.approx(
        -forces * jacobian_z
    )
    assert command.allocation_residual == pytest.approx(np.zeros(3), abs=1.0e-10)


def test_attitude_feedback_uses_opposite_left_right_and_front_rear_forces() -> None:
    controller = QuadrupedVMC(
        mass=20.0,
        mounts=_mounts(),
        height_pid=HeightPID(PIDGains(0.0, 0.0, 0.0)),
        attitude_gains=AttitudeGains(100.0, 0.0, 120.0, 0.0),
        maximum_leg_force=150.0,
    )
    state = VMCState(
        height=0.37,
        vertical_velocity=0.0,
        roll=0.10,
        pitch=-0.10,
        angular_velocity_body=np.zeros(3),
        body_to_world=_rotation(0.10, -0.10),
        leg_angles={name: DEFAULT_GEOMETRY.q_nominal for name in LEG_NAMES},
    )
    command = controller.compute(state, desired_height=0.37, dt=0.002)
    forces = command.leg_forces
    assert forces["front_right"] > forces["front_left"]  # negative restoring roll torque
    assert forces["rear_left"] + forces["rear_right"] > forces["front_left"] + forces["front_right"]
    assert command.desired_body_torque == pytest.approx([-10.0, 12.0])

import numpy as np
import pytest

from ascento_dog.control import (
    DriveCommand,
    PulseTeleop,
    WheelSpeedGains,
    WheelVelocityController,
    wheel_speed_targets,
)

WHEEL_RADIUS = 0.065
MOUNTS_Y = {
    "front_left": 0.20,
    "front_right": -0.20,
    "rear_left": 0.20,
    "rear_right": -0.20,
}


def test_forward_speed_spins_all_wheels_equally() -> None:
    targets = wheel_speed_targets(DriveCommand(v_x=0.5, omega_yaw=0.0), WHEEL_RADIUS, MOUNTS_Y)
    expected = 0.5 / WHEEL_RADIUS
    for name in MOUNTS_Y:
        assert targets[name] == pytest.approx(expected)


def test_pure_yaw_spins_left_and_right_wheels_oppositely() -> None:
    targets = wheel_speed_targets(DriveCommand(v_x=0.0, omega_yaw=1.0), WHEEL_RADIUS, MOUNTS_Y)
    assert targets["front_left"] == pytest.approx(-1.0 * 0.20 / WHEEL_RADIUS)
    assert targets["front_right"] == pytest.approx(1.0 * 0.20 / WHEEL_RADIUS)
    assert targets["front_left"] == pytest.approx(-targets["front_right"])


def test_combined_command_is_linear_superposition() -> None:
    targets = wheel_speed_targets(DriveCommand(v_x=0.5, omega_yaw=1.0), WHEEL_RADIUS, MOUNTS_Y)
    assert targets["front_left"] == pytest.approx((0.5 - 1.0 * 0.20) / WHEEL_RADIUS)
    assert targets["rear_right"] == pytest.approx((0.5 - 1.0 * (-0.20)) / WHEEL_RADIUS)


def test_invalid_radius_rejected() -> None:
    with pytest.raises(ValueError):
        wheel_speed_targets(DriveCommand(0.0, 0.0), 0.0, MOUNTS_Y)


def test_non_finite_command_rejected() -> None:
    with pytest.raises(ValueError):
        wheel_speed_targets(DriveCommand(float("nan"), 0.0), WHEEL_RADIUS, MOUNTS_Y)


def test_proportional_torque_from_speed_error() -> None:
    controller = WheelVelocityController(WheelSpeedGains(kp=2.0, ki=0.0))
    out = controller.update({"w": 5.0}, {"w": 1.0}, 0.01)
    assert out["w"] == pytest.approx(8.0)


def test_output_clamped_to_limit() -> None:
    controller = WheelVelocityController(WheelSpeedGains(kp=2.0, ki=0.0, output_limit=5.0))
    assert controller.update({"w": 100.0}, {"w": 0.0}, 0.01)["w"] == pytest.approx(5.0)


def test_nan_gain_limits_rejected() -> None:
    with pytest.raises(ValueError):
        WheelSpeedGains(1.0, 0.0, integral_limit=float("nan"))


def test_integral_clamped_and_reduced_on_reversal() -> None:
    controller = WheelVelocityController(
        WheelSpeedGains(kp=0.0, ki=1.0, integral_limit=1.0, output_limit=10.0)
    )
    for _ in range(1000):
        controller.update({"w": 1.0}, {"w": 0.0}, 0.01)
    assert controller.update({"w": 1.0}, {"w": 0.0}, 0.01)["w"] == pytest.approx(1.0)
    assert controller.integral["w"] == pytest.approx(1.0)
    controller.update({"w": 0.0}, {"w": 1.0}, 0.01)
    assert controller.integral["w"] < 1.0


def test_anti_windup_freezes_integral_at_output_boundary() -> None:
    controller = WheelVelocityController(
        WheelSpeedGains(kp=0.0, ki=0.5, integral_limit=100.0, output_limit=5.0)
    )
    for _ in range(2000):
        controller.update({"w": 100.0}, {"w": 0.0}, 0.01)
    # integral must stop at the saturation boundary (5.0 / 0.5 = 10), not the integral_limit
    assert controller.integral["w"] == pytest.approx(10.0)
    assert controller.update({"w": 100.0}, {"w": 0.0}, 0.01)["w"] == pytest.approx(5.0)
    # reversing the error unwinds the integral
    controller.update({"w": 0.0}, {"w": 100.0}, 0.01)
    assert controller.integral["w"] < 10.0


def test_wheel_controller_name_mismatch_rejected() -> None:
    controller = WheelVelocityController(WheelSpeedGains(1.0, 0.0))
    with pytest.raises(ValueError):
        controller.update({"a": 1.0}, {"b": 1.0}, 0.01)


def test_press_1_produces_negative_decaying_forward_command() -> None:
    teleop = PulseTeleop()
    teleop.press("1", 10.0)
    assert teleop.command(10.0, forward_speed=0.5, yaw_rate=1.0, decay=1.0) == DriveCommand(-0.5, 0.0)
    assert teleop.command(10.5, forward_speed=0.5, yaw_rate=1.0, decay=1.0) == DriveCommand(-0.25, 0.0)
    assert teleop.command(11.0, forward_speed=0.5, yaw_rate=1.0, decay=1.0) == DriveCommand(0.0, 0.0)


def test_press_2_reverses_forward_axis() -> None:
    teleop = PulseTeleop()
    teleop.press("2", 0.0)
    command = teleop.command(0.0, forward_speed=0.5, yaw_rate=1.0, decay=1.0)
    assert command.v_x == pytest.approx(0.5)
    assert command.omega_yaw == pytest.approx(0.0)


def test_combined_1_3_both_axes() -> None:
    teleop = PulseTeleop()
    teleop.press("1", 0.0)
    teleop.press("3", 0.0)
    command = teleop.command(0.0, forward_speed=0.5, yaw_rate=1.0, decay=1.0)
    assert command.v_x == pytest.approx(-0.5)
    assert command.omega_yaw == pytest.approx(1.0)


def test_repress_restarts_decay_window() -> None:
    teleop = PulseTeleop()
    teleop.press("1", 0.0)
    teleop.press("1", 10.0)
    assert teleop.command(10.5, forward_speed=0.5, yaw_rate=1.0, decay=1.0).v_x == pytest.approx(-0.25)


def test_unpressed_is_zero() -> None:
    teleop = PulseTeleop()
    assert teleop.command(0.0, forward_speed=0.5, yaw_rate=1.0, decay=1.0) == DriveCommand(0.0, 0.0)


def test_non_finite_teleop_inputs_rejected() -> None:
    teleop = PulseTeleop()
    with pytest.raises(ValueError):
        teleop.command(float("nan"), forward_speed=0.5, yaw_rate=1.0, decay=1.0)

import pytest

from ascento_dog.control import (
    HoldTeleop,
    TeleopCommand,
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
    targets = wheel_speed_targets(TeleopCommand(v_x=0.5, omega_yaw=0.0), WHEEL_RADIUS, MOUNTS_Y)
    expected = 0.5 / WHEEL_RADIUS
    for name in MOUNTS_Y:
        assert targets[name] == pytest.approx(expected)


def test_pure_yaw_spins_left_and_right_wheels_oppositely() -> None:
    targets = wheel_speed_targets(TeleopCommand(v_x=0.0, omega_yaw=1.0), WHEEL_RADIUS, MOUNTS_Y)
    assert targets["front_left"] == pytest.approx(-1.0 * 0.20 / WHEEL_RADIUS)
    assert targets["front_right"] == pytest.approx(1.0 * 0.20 / WHEEL_RADIUS)
    assert targets["front_left"] == pytest.approx(-targets["front_right"])


def test_combined_command_is_linear_superposition() -> None:
    targets = wheel_speed_targets(TeleopCommand(v_x=0.5, omega_yaw=1.0), WHEEL_RADIUS, MOUNTS_Y)
    assert targets["front_left"] == pytest.approx((0.5 - 1.0 * 0.20) / WHEEL_RADIUS)
    assert targets["rear_right"] == pytest.approx((0.5 - 1.0 * (-0.20)) / WHEEL_RADIUS)


def test_invalid_radius_rejected() -> None:
    with pytest.raises(ValueError):
        wheel_speed_targets(TeleopCommand(0.0, 0.0), 0.0, MOUNTS_Y)


def test_non_finite_command_rejected() -> None:
    with pytest.raises(ValueError):
        wheel_speed_targets(TeleopCommand(float("nan"), 0.0), WHEEL_RADIUS, MOUNTS_Y)


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


def test_hold_does_not_decay_and_release_stops() -> None:
    teleop = HoldTeleop()
    teleop.press("1")
    # No repeat events are needed to sustain a hold over arbitrarily many updates.
    for _ in range(10000):
        assert teleop.command(forward_speed=0.5, yaw_rate=1.0) == TeleopCommand(0.5, 0.0)
    teleop.release("1")
    assert teleop.command(forward_speed=0.5, yaw_rate=1.0) == TeleopCommand(0.0, 0.0)


@pytest.mark.parametrize(
    "key,expected", [("1", (0.5, 0)), ("2", (-0.5, 0)), ("3", (0, 1)), ("4", (0, -1))]
)
def test_hold_direction_mapping(key, expected) -> None:
    teleop = HoldTeleop()
    teleop.press(key)
    assert teleop.command(forward_speed=0.5, yaw_rate=1.0) == TeleopCommand(*expected)


def test_repeat_opposing_keys_and_independent_release() -> None:
    teleop = HoldTeleop()
    for key in ("1", "1", "2", "3", "4"):
        teleop.press(key)
    assert teleop.command(forward_speed=0.5, yaw_rate=1.0) == TeleopCommand(0, 0)
    teleop.release("2")
    teleop.release("4")
    assert teleop.command(forward_speed=0.5, yaw_rate=1.0) == TeleopCommand(0.5, 1)
    teleop.release("1")
    assert teleop.command(forward_speed=0.5, yaw_rate=1.0) == TeleopCommand(0, 1)
    teleop.reset()
    teleop.release("3")
    teleop.press("W")
    assert teleop.command(forward_speed=0.5, yaw_rate=1.0) == TeleopCommand(0, 0)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -0.1])
@pytest.mark.parametrize("parameter", ["forward_speed", "yaw_rate"])
def test_invalid_teleop_inputs_rejected(parameter, value) -> None:
    inputs = dict(forward_speed=0.5, yaw_rate=1.0)
    inputs[parameter] = value
    with pytest.raises(ValueError):
        HoldTeleop().command(**inputs)

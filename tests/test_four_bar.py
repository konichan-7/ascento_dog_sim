from math import radians

import numpy as np
import pytest

from ascento_dog.kinematics import DEFAULT_GEOMETRY, UnreachableTargetError


def test_user_dimensions_are_preserved_in_si_units() -> None:
    geometry = DEFAULT_GEOMETRY
    assert (geometry.l1, geometry.l2, geometry.l3, geometry.l4, geometry.l23) == (
        0.235,
        0.238,
        0.244,
        0.10889,
        0.057,
    )


@pytest.mark.parametrize("q", np.linspace(radians(-65), radians(-15), 41))
def test_forward_kinematics_closes_every_bar(q: float) -> None:
    geometry = DEFAULT_GEOMETRY
    pose = geometry.forward(float(q))
    assert max(abs(value) for value in geometry.bar_residuals(pose).values()) < 1.0e-12
    first = pose.d - pose.b
    second = pose.c - pose.d
    assert first[0] * second[1] - first[1] * second[0] > 0.0


@pytest.mark.parametrize("q", np.linspace(radians(-65), radians(-15), 41))
def test_inverse_round_trip(q: float) -> None:
    geometry = DEFAULT_GEOMETRY
    wheel = geometry.forward(float(q)).e
    recovered = geometry.inverse(wheel)
    assert recovered == pytest.approx(q, abs=1.0e-10)


@pytest.mark.parametrize("q", [radians(-65), radians(-40), radians(-15)])
def test_height_inverse_round_trip(q: float) -> None:
    geometry = DEFAULT_GEOMETRY
    z = float(geometry.forward(q).e[1])
    recovered = geometry.inverse_height(z)
    assert recovered == pytest.approx(q, abs=2.0e-9)


def test_off_curve_target_is_rejected() -> None:
    geometry = DEFAULT_GEOMETRY
    target = geometry.forward(geometry.q_nominal).e + np.array([0.01, 0.0])
    with pytest.raises(UnreachableTargetError, match="one-DoF linkage path"):
        geometry.inverse(target)


def test_working_path_is_near_vertical() -> None:
    geometry = DEFAULT_GEOMETRY
    wheel_positions = np.array(
        [geometry.forward(float(q)).e for q in np.linspace(geometry.q_min, geometry.q_max, 101)]
    )
    vertical_travel = float(np.ptp(wheel_positions[:, 1]))
    horizontal_excursion = float(np.ptp(wheel_positions[:, 0]))
    assert vertical_travel > 0.30
    assert horizontal_excursion < 0.04


@pytest.mark.parametrize("q", np.linspace(radians(-65), radians(-15), 11))
def test_analytical_wheel_jacobian_matches_centered_difference(q: float) -> None:
    geometry = DEFAULT_GEOMETRY
    step = 1.0e-6
    numerical = (
        geometry.forward(float(q) + step, check_limits=False).e
        - geometry.forward(float(q) - step, check_limits=False).e
    ) / (2.0 * step)
    analytical = geometry.wheel_jacobian(float(q))
    assert analytical == pytest.approx(numerical, abs=2.0e-9)

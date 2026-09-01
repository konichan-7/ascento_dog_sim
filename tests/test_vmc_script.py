import argparse

import pytest

from ascento_dog.scripts import vmc


def _args(**overrides: object) -> argparse.Namespace:
    base = dict(
        period=8.0,
        amplitude=0.03,
        duration=12.0,
        disturbance=45.0,
        disturbance_period=4.0,
        disturbance_duration=0.15,
        height=0.37,
        headless=False,
        teleop=False,
        forward_speed=0.5,
        yaw_rate=1.0,
        decay=1.0,
    )
    base.update(overrides)
    return argparse.Namespace(**base)


def test_teleop_requires_viewer() -> None:
    with pytest.raises(SystemExit, match="teleop 需要 viewer"):
        vmc._validate_args(_args(teleop=True, headless=True))


def test_nonpositive_drive_parameters_rejected() -> None:
    with pytest.raises(SystemExit, match="forward-speed"):
        vmc._validate_args(_args(forward_speed=-0.1))
    with pytest.raises(SystemExit, match="decay"):
        vmc._validate_args(_args(decay=0.0))


def test_valid_teleop_args_pass() -> None:
    vmc._validate_args(_args(teleop=True))


def test_teleop_ignores_amplitude_in_height_check() -> None:
    # teleop 不走高度正弦,幅值即使超出非 teleop 的叠加范围也不应报错。
    vmc._validate_args(_args(teleop=True, amplitude=0.2))

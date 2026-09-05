import argparse

import pytest

from ascento_dog.scripts import vmc
from ascento_dog.simulation.viewer import make_esc_exit_callback


def _args(**overrides: object) -> argparse.Namespace:
    base = dict(
        period=8.0,
        amplitude=0.03,
        disturbance=45.0,
        disturbance_period=4.0,
        disturbance_duration=0.15,
        height=0.37,
        teleop=False,
        forward_speed=0.5,
        yaw_rate=1.0,
    )
    base.update(overrides)
    return argparse.Namespace(**base)


def test_nonpositive_teleop_parameters_rejected() -> None:
    with pytest.raises(SystemExit, match="forward-speed"):
        vmc._validate_args(_args(forward_speed=-0.1))
    with pytest.raises(SystemExit, match="yaw-rate"):
        vmc._validate_args(_args(yaw_rate=float("nan")))


def test_valid_teleop_args_pass() -> None:
    vmc._validate_args(_args(teleop=True))


def test_default_teleop_yaw_mapping_is_four_rad_per_second(monkeypatch) -> None:
    monkeypatch.setattr("sys.argv", ["vmc"])
    assert vmc.parse_args().yaw_rate == pytest.approx(4.0)


def test_teleop_ignores_amplitude_in_height_check() -> None:
    # teleop 不走高度正弦,幅值即使超出非 teleop 的叠加范围也不应报错。
    vmc._validate_args(_args(teleop=True, amplitude=0.2))


def test_esc_callback_sets_close_event() -> None:
    should_close, key_callback = make_esc_exit_callback()
    assert not should_close.is_set()
    key_callback(256)  # ESC (GLFW_KEY_ESCAPE)
    assert should_close.is_set()


def test_esc_callback_ignores_non_esc_keys() -> None:
    should_close, key_callback = make_esc_exit_callback()
    key_callback(ord("1"))
    key_callback(ord("W"))
    assert not should_close.is_set()


def test_teleop_uses_native_viewer_with_both_panels(monkeypatch) -> None:
    from types import SimpleNamespace

    import mujoco.viewer

    from ascento_dog.control import HoldTeleop, TeleopCommand
    from ascento_dog.simulation.teleop_viewer import NativeTeleopInput

    calls = {}

    class Viewer:
        viewport = SimpleNamespace(left=200, bottom=0, width=600, height=600)

        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def is_running(self):
            return False

    def launch(*args, **kwargs):
        calls.update(kwargs)
        return Viewer()

    monkeypatch.setattr(mujoco.viewer, "launch_passive", launch)
    teleop = HoldTeleop()
    teleop.press("1")
    vmc._run_with_viewer(None, None, None, _args(teleop=True), 0, teleop, None)
    assert calls["show_left_ui"] is True
    assert calls["show_right_ui"] is True
    assert isinstance(calls["key_callback"].__self__, NativeTeleopInput)
    assert teleop.command(forward_speed=1, yaw_rate=4) == TeleopCommand(0, 0)

"""Native UI callback chaining and held-key behavior without opening a GUI."""

from threading import Event
from types import SimpleNamespace

import glfw
import pytest

from ascento_dog.control import HoldTeleop, TeleopCommand
from ascento_dog.simulation.teleop_viewer import NativeTeleopInput


@pytest.fixture
def keyboard(monkeypatch):
    result = NativeTeleopInput(HoldTeleop(), Event())
    result.viewport = lambda: SimpleNamespace(left=200, bottom=0, width=600, height=600)
    monkeypatch.setattr(glfw, "get_window_attrib", lambda *_: True)
    monkeypatch.setattr(glfw, "get_window_size", lambda *_: (1000, 600))
    monkeypatch.setattr(glfw, "get_framebuffer_size", lambda *_: (1000, 600))
    monkeypatch.setattr(glfw, "get_cursor_pos", lambda *_: (500, 300))
    monkeypatch.setattr(glfw, "get_key", lambda *_: glfw.RELEASE)
    return result


def command(keyboard):
    return keyboard.teleop.command(forward_speed=1.0, yaw_rate=4.0)


def test_hold_release_and_native_shortcuts(keyboard):
    forwarded = []
    keyboard._native_key = lambda *args: forwarded.append(args[1:])
    keyboard._on_key(None, glfw.KEY_1, 0, glfw.PRESS, 0)
    keyboard._on_key(None, glfw.KEY_1, 0, glfw.REPEAT, 0)
    assert command(keyboard) == TeleopCommand(1, 0)
    assert not forwarded  # Drive presses must not toggle native geom groups.
    keyboard._on_key(None, glfw.KEY_1, 0, glfw.RELEASE, 0)
    assert command(keyboard) == TeleopCommand(0, 0)
    for key in (glfw.KEY_TAB, glfw.KEY_F1, glfw.KEY_F4, glfw.KEY_F):
        keyboard._on_key(None, key, 0, glfw.PRESS, 0)
        assert forwarded[-1] == (key, 0, glfw.PRESS, 0)


def test_panels_and_modifier_keys_are_forwarded_release_still_stops(keyboard, monkeypatch):
    forwarded = []
    keyboard._native_key = lambda *args: forwarded.append(args[1])
    keyboard._on_key(None, glfw.KEY_1, 0, glfw.PRESS, 0)
    monkeypatch.setattr(glfw, "get_cursor_pos", lambda *_: (100, 300))
    keyboard._on_key(None, glfw.KEY_1, 0, glfw.RELEASE, 0)
    keyboard._on_key(None, glfw.KEY_2, 0, glfw.PRESS, 0)
    assert command(keyboard) == TeleopCommand(0, 0)
    assert forwarded == [glfw.KEY_1, glfw.KEY_2]
    monkeypatch.setattr(glfw, "get_cursor_pos", lambda *_: (500, 300))
    keyboard._on_key(None, glfw.KEY_3, 0, glfw.PRESS, glfw.MOD_SHIFT)
    assert command(keyboard) == TeleopCommand(0, 0)
    assert forwarded[-1] == glfw.KEY_3


def test_focus_loss_resets_and_preserves_native_focus_callback(keyboard):
    focused = []
    keyboard._native_focus = lambda w, value: focused.append(value)
    for key in (glfw.KEY_1, glfw.KEY_3):
        keyboard._on_key(None, key, 0, glfw.PRESS, 0)
    assert command(keyboard) == TeleopCommand(1, 4)
    keyboard._on_focus(None, False)
    keyboard._on_focus(None, True)
    keyboard._on_key(None, glfw.KEY_1, 0, glfw.REPEAT, 0)
    assert command(keyboard) == TeleopCommand(0, 0)
    assert focused == [False, True]


def test_bootstrap_restores_first_native_group_toggle_and_keeps_c_callbacks(keyboard, monkeypatch):
    window = object()
    monkeypatch.setattr(glfw, "get_current_context", lambda: window)
    groups = {glfw.KEY_1: True}
    hooks = {}

    def native_key(w, key, scan, action, mods):
        if action == glfw.PRESS:
            groups[key] = not groups[key]
            keyboard.key_callback(key)

    def install_key(w, callback):
        assert w is window
        hooks["key"] = callback
        return native_key

    monkeypatch.setattr(glfw._glfw, "glfwSetKeyCallback", install_key)
    monkeypatch.setattr(glfw._glfw, "glfwSetWindowFocusCallback", lambda *args: None)
    monkeypatch.setattr(glfw._glfw, "glfwSetWindowCloseCallback", lambda *args: None)
    native_key(window, glfw.KEY_1, 0, glfw.PRESS, 0)
    keyboard.check_error()
    assert groups[glfw.KEY_1] is True
    assert command(keyboard) == TeleopCommand(1, 0)
    assert hooks["key"] is keyboard._key_hook
    keyboard._on_key(window, glfw.KEY_1, 0, glfw.RELEASE, 0)
    assert command(keyboard) == TeleopCommand(0, 0)


def test_no_window_fails_explicitly_and_stops(keyboard, monkeypatch):
    keyboard.teleop.press("1")
    monkeypatch.setattr(glfw, "get_current_context", lambda: None)
    keyboard.key_callback(glfw.KEY_1)
    assert command(keyboard) == TeleopCommand(0, 0)
    assert keyboard.should_close.is_set()
    with pytest.raises(RuntimeError, match="键盘输入失败"):
        keyboard.check_error()


def test_retina_viewport_and_unfocused_input(keyboard, monkeypatch):
    monkeypatch.setattr(glfw, "get_framebuffer_size", lambda *_: (2000, 1200))
    keyboard.viewport = lambda: SimpleNamespace(left=400, bottom=0, width=1200, height=1200)
    assert keyboard._over_scene(None)
    monkeypatch.setattr(glfw, "get_window_attrib", lambda *_: False)
    assert not keyboard._over_scene(None)


@pytest.mark.parametrize("method", ["escape", "close", "stop"])
def test_shutdown_clears_commands(keyboard, method):
    keyboard.teleop.press("1")
    if method == "escape":
        keyboard.key_callback(glfw.KEY_ESCAPE)
    elif method == "close":
        keyboard._on_close(None)
    else:
        keyboard.stop()
    assert command(keyboard) == TeleopCommand(0, 0)

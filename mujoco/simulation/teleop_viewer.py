"""Held-key input added to the native MuJoCo viewer without replacing its UI."""

from __future__ import annotations

from collections.abc import Callable
from threading import Event

import glfw

from ascento_dog.control import HoldTeleop


class NativeTeleopInput:
    """Chain native GLFW callbacks on the viewer's own UI thread.

    Pass ``key_callback`` to launch_passive, then provide ``viewer.viewport``
    through ``viewport``. Unmodified 1/2/3/4 over the 3D viewport drive the robot;
    panel input and all other keys retain native behavior. Releases and focus
    loss clear commands. Callback failures are reported by ``check_error``.

    The first native key callback supplies its current GLFW context, allowing
    installation on the correct thread (also under mjpython on macOS). We use
    GLFW's C callback setters because the Python wrappers discard callbacks
    previously installed by C++. The native viewer retains our bound callback,
    which keeps these ctypes callbacks alive until its window is destroyed.
    """

    def __init__(self, teleop: HoldTeleop, should_close: Event) -> None:
        self.teleop = teleop
        self.should_close = should_close
        self.viewport: Callable[[], object] | None = None
        self._window = None
        self._enabled = True
        self._replaying = False
        self._error: Exception | None = None
        self._native_key = None
        self._native_focus = None
        self._native_close = None
        self._key_hook = glfw._GLFWkeyfun(self._on_key)
        self._focus_hook = glfw._GLFWwindowfocusfun(self._on_focus)
        self._close_hook = glfw._GLFWwindowclosefun(self._on_close)

    def key_callback(self, keycode: int) -> None:
        """Native viewer callback; installs release hooks on the first key."""
        if self._replaying or not self._enabled:
            return
        try:
            if keycode == glfw.KEY_ESCAPE:
                self.stop()
                self.should_close.set()
                return
            if self._window is not None:
                return  # Subsequent teleop events are handled by the raw hook.
            window = glfw.get_current_context()
            if not window:
                raise RuntimeError("MuJoCo 查看器未提供当前 GLFW 窗口，无法接入松开事件")
            self._window = window
            self._native_key = glfw._glfw.glfwSetKeyCallback(window, self._key_hook)
            self._native_focus = glfw._glfw.glfwSetWindowFocusCallback(window, self._focus_hook)
            self._native_close = glfw._glfw.glfwSetWindowCloseCallback(window, self._close_hook)
            if (
                self._is_direction(keycode)
                and self._over_scene(window)
                and not self._modifiers(window)
            ):
                # The first press has already reached MuJoCo's numeric geom-group
                # shortcut. Replay it once to undo that toggle, without recursion.
                self._replaying = True
                try:
                    self._native_key(window, keycode, 0, glfw.PRESS, 0)
                finally:
                    self._replaying = False
                self.teleop.press(chr(keycode))
        except Exception as error:
            self._fail(error)

    def stop(self) -> None:
        """Disable teleop and clear input; native UI retains callback ownership."""
        self._enabled = False
        self.teleop.reset()

    def check_error(self) -> None:
        """Raise callback failures on the simulation thread instead of losing them."""
        if self._error is not None:
            raise RuntimeError("原生查看器键盘输入失败") from self._error

    def _fail(self, error: Exception) -> None:
        self._error = error
        self.stop()
        self.should_close.set()

    @staticmethod
    def _is_direction(key: int) -> bool:
        return glfw.KEY_1 <= key <= glfw.KEY_4

    @staticmethod
    def _modifiers(window) -> bool:
        return any(
            glfw.get_key(window, key) == glfw.PRESS
            for key in (
                glfw.KEY_LEFT_SHIFT,
                glfw.KEY_RIGHT_SHIFT,
                glfw.KEY_LEFT_CONTROL,
                glfw.KEY_RIGHT_CONTROL,
                glfw.KEY_LEFT_ALT,
                glfw.KEY_RIGHT_ALT,
                glfw.KEY_LEFT_SUPER,
                glfw.KEY_RIGHT_SUPER,
            )
        )

    def _over_scene(self, window) -> bool:
        if self.viewport is None or not glfw.get_window_attrib(window, glfw.FOCUSED):
            return False
        rect = self.viewport()
        width, height = glfw.get_window_size(window)
        fb_width, fb_height = glfw.get_framebuffer_size(window)
        if width <= 0 or height <= 0:
            return False
        x, y = glfw.get_cursor_pos(window)
        x, y = x * fb_width / width, (height - y) * fb_height / height
        return (
            rect.left <= x < rect.left + rect.width and rect.bottom <= y < rect.bottom + rect.height
        )

    def _on_key(self, window, key: int, scancode: int, action: int, mods: int) -> None:
        try:
            if self._enabled and self._is_direction(key):
                if action == glfw.RELEASE:
                    self.teleop.release(chr(key))  # Also release while hovering a UI panel.
                elif mods == 0 and self._over_scene(window):
                    if action == glfw.PRESS:
                        self.teleop.press(chr(key))
                    return  # Reserve drive keys; do not toggle native geom groups.
            if self._native_key:
                self._native_key(window, key, scancode, action, mods)
        except Exception as error:
            self._fail(error)

    def _on_focus(self, window, focused: int) -> None:
        try:
            if not focused:
                self.teleop.reset()
            if self._native_focus:
                self._native_focus(window, focused)
        except Exception as error:
            self._fail(error)

    def _on_close(self, window) -> None:
        self.stop()
        self.should_close.set()
        try:
            if self._native_close:
                self._native_close(window)
        except Exception as error:
            self._fail(error)

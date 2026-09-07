"""MuJoCo 原生查看器的跨平台启动辅助函数。"""

from __future__ import annotations

import os
import sys
import threading
from collections.abc import Callable
from pathlib import Path

_ESC_KEYCODE = 256  # GLFW_KEY_ESCAPE


def ensure_mjpython_on_macos(module: str) -> None:
    """在 macOS 上通过 ``mjpython`` 重新启动指定模块。"""

    if sys.platform != "darwin" or "MJPYTHON_BIN" in os.environ:
        return

    mjpython = Path(sys.executable).with_name("mjpython")
    if not mjpython.is_file():
        raise RuntimeError(
            "macOS 上的 MuJoCo Viewer 需要 mjpython，但未在 "
            f"{mjpython} 找到。请运行 `uv sync --dev` 重新安装依赖。"
        )
    os.execv(str(mjpython), [str(mjpython), "-m", module, *sys.argv[1:]])


def make_esc_exit_callback() -> tuple[threading.Event, Callable[[int], None]]:
    """返回 ``(should_close, key_callback)`` 以在 viewer 中按 ESC 退出。

    把 ``key_callback`` 传给 ``launch_passive``,并用
    ``not should_close.is_set()`` 门控渲染循环。ESC(GLFW 键码 256)
    置位事件,使脚本在按 ESC 或关闭窗口时结束。
    """

    should_close = threading.Event()

    def key_callback(keycode: int) -> None:
        if keycode == _ESC_KEYCODE:
            should_close.set()

    return should_close, key_callback

"""MuJoCo 原生查看器的跨平台启动辅助函数。"""

from __future__ import annotations

import os
import sys
from pathlib import Path


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

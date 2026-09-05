# Ascento Dog 仿真

## 环境配置

需要 Python 3.11–3.14 和 [uv](https://docs.astral.sh/uv/)。在仓库根目录执行：

```bash
uv sync --dev
uv run pytest
```

三个可视化入口需要桌面 OpenGL 环境；macOS 会自动改用 MuJoCo 的 `mjpython` 启动器。

## 启动脚本

```bash
# 单腿悬空运动学
uv run leg-kinematics

# 整车 VMC；加 --teleop 后按住数字键 1/2/3/4 前进、后退、左转、右转，松开停止
uv run vmc
uv run vmc --teleop

# 200 mm 台阶；默认打开查看器，--headless 用于无界面验证
uv run cross-step
uv run cross-step --headless --duration 30
```

遥控模式保留原生 MuJoCo 查看器。左侧 `Rendering → Contact force` 显示接触力，
`Contact point` 显示接触点；`Tab` / `Shift+Tab` 显示或隐藏左右面板。
遥控时将鼠标移到中央三维视图区；鼠标在面板上时，数字键保留原生 UI 输入行为。

所有脚本按 `Esc` 或关闭窗口退出。坐标系、解析运动学、VMC、控制框架和跨台阶设计统一记录在 [docs/design.md](docs/design.md)。

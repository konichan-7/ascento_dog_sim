# Ascento Dog 仿真

## 环境配置

需要 Python 3.11–3.14。以下命令均在仓库根目录（包含 `pyproject.toml` 的目录）执行。
虚拟环境 `.venv` 用来隔离本项目的 Python 和依赖；首次安装选择下面一种方式即可。

### 方式一：使用 uv（推荐）

安装 [uv](https://docs.astral.sh/uv/) 后执行：

```bash
uv sync --dev
uv run pytest
```

`uv sync --dev` 会自动创建 `.venv`，按 `uv.lock` 安装项目与开发依赖。
`uv run` 会使用这个环境，不需要手动激活；也可以按下文激活后直接使用 `python` 启动。

### 方式二：使用 Python 自带的 venv

先确认用于创建环境的 Python 版本在上述范围内。macOS / Linux（bash、zsh）：

```bash
python3 --version
python3 -m venv .venv
source .venv/bin/activate
```

Windows PowerShell：

```powershell
python --version
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

激活后安装项目与开发依赖：

```bash
python -m pip install --upgrade pip
python -m pip install -e . --group dev
```

`-e .` 使用当前仓库源码，修改代码后可直接重新运行；`--group dev` 安装
`pyproject.toml` 中声明的 pytest、Ruff 等开发依赖，见 [pip 依赖组说明](https://pip.pypa.io/en/stable/user_guide/#dependency-groups)。
这种方式按 `pyproject.toml` 的版本范围安装，不读取 `uv.lock`；复现锁定依赖版本时使用方式一。

### 日常使用：激活、检查和退出环境

每次打开新终端后，进入仓库根目录并激活已有环境，无需重新创建：

```bash
# macOS / Linux（bash、zsh）
source .venv/bin/activate
```

```powershell
# Windows PowerShell
.\.venv\Scripts\Activate.ps1
```

检查当前 Python 是否来自本项目的 `.venv`：

```bash
python -c "import sys; print(sys.executable)"
```

输出应指向仓库内的 `.venv/bin/python`（macOS / Linux）或 `.venv\Scripts\python.exe`（Windows）。
VS Code 中可执行 `Python: Select Interpreter` 选择同一解释器。
用完后运行 `deactivate` 退出环境；这不会删除环境或已安装的依赖。

也可以直接指定虚拟环境解释器，省略激活步骤，例如
`.venv/bin/python -m ascento_dog.scripts.vmc`；Windows 使用
`.\.venv\Scripts\python.exe -m ascento_dog.scripts.vmc`。
若 PowerShell 不允许执行激活脚本，也可用这个解释器路径代替安装和启动命令中的 `python`。
激活脚本与解释器调用方式见 [Python venv 文档](https://docs.python.org/3/library/venv.html)。

三个可视化入口需要桌面 OpenGL 环境；macOS 入口会自动改用同一虚拟环境中的
MuJoCo `mjpython` 启动器，无需手动切换。无图形界面时可运行下文的 `cross-step --headless`。

## 启动脚本

使用 uv 时，无需激活环境：

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

如果已激活 `.venv`，可以直接通过 Python 模块启动（两种安装方式均适用）：

```bash
# 单腿悬空运动学
python -m ascento_dog.scripts.leg_kinematics

# 整车 VMC / 键盘遥控
python -m ascento_dog.scripts.vmc
python -m ascento_dog.scripts.vmc --teleop

# 台阶查看器 / 无界面验证
python -m ascento_dog.scripts.cross_step
python -m ascento_dog.scripts.cross_step --headless --duration 30

# 运行测试
python -m pytest
```

遥控模式保留原生 MuJoCo 查看器。左侧 `Rendering → Contact force` 显示接触力，
`Contact point` 显示接触点；`Tab` / `Shift+Tab` 显示或隐藏左右面板。
遥控时将鼠标移到中央三维视图区；鼠标在面板上时，数字键保留原生 UI 输入行为。

所有脚本按 `Esc` 或关闭窗口退出。坐标系、解析运动学、VMC、控制框架和跨台阶设计统一记录在 [docs/design.md](docs/design.md)。

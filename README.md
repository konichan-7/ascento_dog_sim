# Ascento Dog 仿真

本仓库用于构建采用 Ascento 单自由度腿部连杆的四轮足机器人仿真与控制系统。

当前里程碑包括：

- 在 `AGENTS.md` 中明确连杆约定和项目开发规则；
- 单腿解析正运动学与逆运动学；
- 带真实闭环约束和默认地板的浮空 MuJoCo 模型；
- 对比解析点位与 MuJoCo 站点的数值测试；
- 使用四条闭环腿、四个独立轮关节和长方体机身构成的完整机器人模型。
- 基于解析雅可比的 VMC、整车重力补偿、底盘高度 PID 和 roll/pitch 抗扰控制。
- 键盘遥杆驱动（`vmc --teleop`，数字键 1/2/3/4）：轮速 PI 闭环，前后平移与原地转向。

## 快速开始

```bash
uv sync --dev
uv run pytest
uv run leg-kinematics
uv run vmc
```

模型查看命令需要桌面 OpenGL 环境；测试与验证命令可以无界面运行。在 macOS 上，两个查看命令都会自动使用 MuJoCo 提供的 `mjpython` 启动器，以满足 Cocoa 主线程要求。

`leg-kinematics` 显示零重力、无接触的单腿悬空运动学；`vmc` 运行启用重力和四轮地面接触的整车闭环控制。数值验证统一由 `pytest` 负责，不再维护与测试重复的独立验证脚本。加 `--teleop` 可用键盘数字键 1/2/3/4 驱动整车前后平移与原地转向（高度/姿态仍由 VMC 稳定，ESC 退出），详见 `docs/teleop.md`。

坐标约定、方程、假设和验证标准详见 `docs/leg_kinematics.md`。

完整四轮足模型的布局、示意尺寸和局限详见 `docs/quadruped.md`。

VMC 虚功推导、高度 PID、任意 roll/pitch 下的四腿力分配和当前示意增益详见 `docs/vmc.md`。

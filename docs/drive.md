# 轮速驱动与 WASD 遥杆

## 目的

在整车 VMC 高度/姿态稳定之上,用键盘 W/A/S/D 控制前后平移与原地转向。

## 驱动运动学

车体系 +x 指前。车轮只沿 x 滚动,轮心线速度 `v_xi = v_x − ω_yaw·y_i`。
经 MuJoCo 集成测试实测,本模型**正自转(绕 +y)= 前进**,因此轮角速度目标(rad/s):

    ω_wheel_i = (v_x − ω_yaw·y_i) / R,    R = 0.065 m

- W/S:仅 `v_x` → 四轮同速,前后平移。
- A/D:仅 `ω_yaw` → 左右轮反向(`y = ±0.20`)→ 原地转向(A = 左转)。
- W+A 组合:圆弧前进。

## 轮速 PI

`ascento_dog/control/wheel_speed.py` 的 `WheelVelocityController` 每轮独立 PI,
误差积分 + 条件抗饱和 + 输出限幅。示意增益 `kp=0.5, ki=0.2, integral_limit=3.0,
output_limit=10.0`(待整定)。

## 键盘交互

viewer 的 `key_callback` 只收到按下事件(无松开、无长按重复),故采用
`PulseTeleop`:按一次锁存方向,约 `--decay`(默认 1 s)内线性衰减到零,再按重新锁存。
时间基准为 `time.monotonic()`(墙钟),避免在 UI 线程读取仿真状态。

## 使用

    uv run vmc --teleop
    uv run vmc --teleop --forward-speed 0.5 --yaw-rate 1.0 --decay 1.0

`--teleop` 需要 viewer(桌面 OpenGL),不能与 `--headless` 同时使用。按 W/S 前进/后退,
A/D 原地转向;高度与姿态由 VMC 保持。姿态曲线在演示结束后照常导出。

## 复现验证

    uv sync --dev
    uv run pytest
    uv run vmc --teleop

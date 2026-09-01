# 轮速驱动与数字键遥杆

## 目的

在整车 VMC 高度/姿态稳定之上,用键盘数字键 1/2/3/4 控制前后平移与原地转向。

## 驱动运动学

车体系 +x 指前。车轮只沿 x 滚动,轮心线速度 `v_xi = v_x − ω_yaw·y_i`。
经 MuJoCo 集成测试实测,本模型**正自转(绕 +y)= 前进**,因此轮角速度目标(rad/s):

    ω_wheel_i = (v_x − ω_yaw·y_i) / R,    R = 0.065 m

- 仅 `v_x`:四轮同速 → 前后平移。
- 仅 `ω_yaw`:左右轮反向(`y = ±0.20`)→ 原地转向。
- 两者组合:圆弧前进。

## 轮速 PI

`ascento_dog/control/wheel_speed.py` 的 `WheelVelocityController` 每轮独立 PI,
误差积分 + 条件抗饱和 + 输出限幅。示意增益 `kp=0.5, ki=0.2, integral_limit=3.0,
output_limit=10.0`(待整定)。

## 键盘交互

数字键映射:`1`=前进、`2`=后退、`3`=左转、`4`=右转,可组合(如同时按 1 和 3)。

**前进方向**:默认相机在 azimuth 135° 看的是机器人背面(+x 前腿朝远处),操作者看到的
"前方"是近侧(车体系 -x)。为匹配操作直觉,`1`(前进)驱动车体系 **-x**(即 `v_x`
为负),`2`(后退)驱动 +x;`3`(左转)对应 `ω_yaw` 为正,`4`(右转)为负。

viewer 的 `key_callback` 只收到按下事件(无松开、无长按重复),故采用
`PulseTeleop`:按一次锁存方向,约 `--decay`(默认 1 s)内线性衰减到零,再按重新锁存。
时间基准为 `time.monotonic()`(墙钟),避免在 UI 线程读取仿真状态。遥杆模式隐藏左右
UI 面板,避免文本输入框抢键盘焦点。

## 使用

    uv run vmc --teleop
    uv run vmc --teleop --forward-speed 1.0 --yaw-rate 2.0 --decay 1.0

`--teleop` 需要 viewer(桌面 OpenGL)。按数字键 1/2/3/4 驱动(1前 2后 3左 4右),按 ESC 键
结束程序并关闭 viewer;程序无时间上限,运行到 ESC 或关闭窗口为止。`--forward-speed`
(默认 1.0 m/s)与 `--yaw-rate`(默认 2.0 rad/s)为脉冲幅值,可按需调大调小。

## 复现验证

    uv sync --dev
    uv run pytest
    uv run vmc --teleop

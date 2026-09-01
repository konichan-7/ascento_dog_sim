"""运行整车 VMC 演示;--teleop 用数字键 1/2/3/4 驱动,viewer 内叠加显示 IMU 姿态。"""

from __future__ import annotations

import argparse
import threading
import time
from math import pi, sin

import mujoco
import numpy as np

from ascento_dog.control import PulseTeleop, WheelSpeedGains, WheelVelocityController
from ascento_dog.kinematics import DEFAULT_GEOMETRY
from ascento_dog.simulation import (
    apply_body_disturbance,
    create_default_vmc,
    load_quadruped_model,
    read_imu_attitude,
    set_quadruped_pose,
    step_drive,
    step_vmc,
)
from ascento_dog.simulation.viewer import ensure_mjpython_on_macos

_ESC_KEYCODE = 256  # GLFW_KEY_ESCAPE


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--height", type=float, default=0.37, help="平均目标底盘高度，m")
    parser.add_argument("--amplitude", type=float, default=0.03, help="高度正弦幅值，m")
    parser.add_argument("--period", type=float, default=8.0, help="高度变化周期，s")
    parser.add_argument("--duration", type=float, default=12.0, help="演示总时长，s")
    parser.add_argument("--disturbance", type=float, default=45.0, help="扰动基准力矩，N·m")
    parser.add_argument("--disturbance-period", type=float, default=4.0, help="扰动周期，s")
    parser.add_argument(
        "--disturbance-duration", type=float, default=0.15, help="单次扰动持续时间，s"
    )
    parser.add_argument("--headless", action="store_true", help="不打开 MuJoCo Viewer")
    parser.add_argument("--teleop", action="store_true", help="启用键盘遥杆(1前 2后 3左 4右)")
    parser.add_argument("--forward-speed", type=float, default=0.5, help="前进/后退脉冲幅值, m/s")
    parser.add_argument("--yaw-rate", type=float, default=1.0, help="转向脉冲幅值, rad/s")
    parser.add_argument("--decay", type=float, default=1.0, help="脉冲线性衰减时长, s")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _validate_args(args)
    if not args.headless:
        ensure_mjpython_on_macos("ascento_dog.scripts.vmc")

    model, data = load_quadruped_model()
    set_quadruped_pose(model, data, DEFAULT_GEOMETRY.q_nominal, wheels_on_floor=True)
    controller = create_default_vmc(model)
    wheel_controller = (
        WheelVelocityController(
            WheelSpeedGains(
                kp=0.5, ki=0.2, integral_limit=3.0, output_limit=10.0  # 示意,待整定
            )
        )
        if args.teleop
        else None
    )
    teleop = PulseTeleop() if args.teleop else None
    simulation_start = float(data.time)
    last_overlay = -10.0

    if args.headless:
        while data.time - simulation_start < args.duration:
            last_overlay = _step(
                model, data, controller, args, simulation_start,
                teleop, wheel_controller, last_overlay, viewer=None,
            )
    else:
        _run_with_viewer(
            model, data, controller, args, simulation_start,
            teleop, wheel_controller, last_overlay,
        )


def _run_with_viewer(
    model, data, controller, args, simulation_start, teleop, wheel_controller, last_overlay
) -> None:
    import mujoco.viewer

    should_close = threading.Event()

    def key_callback(keycode: int) -> None:
        if keycode == _ESC_KEYCODE:
            should_close.set()
            return
        key = chr(keycode) if 32 <= keycode < 127 else ""
        if teleop is not None and key and key in "1234":
            teleop.press(key, time.monotonic())

    # teleop 模式下隐藏左右 UI 面板,避免文本输入框抢键盘焦点与数字键遥杆冲突。
    show_ui = teleop is None
    with mujoco.viewer.launch_passive(
        model,
        data,
        key_callback=key_callback,
        show_left_ui=show_ui,
        show_right_ui=show_ui,
    ) as viewer:
        while (
            viewer.is_running()
            and not should_close.is_set()
            and data.time - simulation_start < args.duration
        ):
            step_start = time.monotonic()
            last_overlay = _step(
                model, data, controller, args, simulation_start,
                teleop, wheel_controller, last_overlay, viewer,
            )
            viewer.sync()
            remaining = float(model.opt.timestep) - (time.monotonic() - step_start)
            if remaining > 0.0:
                time.sleep(remaining)


def _step(
    model, data, controller, args, simulation_start, teleop, wheel_controller, last_overlay, viewer
) -> float:
    elapsed = float(data.time) - simulation_start
    if teleop is not None:
        command = teleop.command(
            time.monotonic(),
            forward_speed=args.forward_speed,
            yaw_rate=args.yaw_rate,
            decay=args.decay,
        )
        step_drive(model, data, controller, wheel_controller, command, desired_height=args.height)
    else:
        desired_height = args.height + args.amplitude * sin(2.0 * pi * elapsed / args.period)
        phase = elapsed % args.disturbance_period
        start = 0.5 * args.disturbance_period
        active = start <= phase < start + args.disturbance_duration
        magnitude = args.disturbance if active else 0.0

        # 同时激励三轴;yaw 当前没有闭环控制,因此其残余偏角会保留在 IMU 读数上。
        torque_world = (magnitude, -(2.0 / 3.0) * magnitude, magnitude / 9.0)
        apply_body_disturbance(model, data, torque_world=torque_world)
        step_vmc(model, data, controller, desired_height=desired_height)

    return _update_attitude(model, data, viewer, last_overlay)


def _update_attitude(model, data, viewer, last_overlay: float) -> float:
    """约 10 Hz 更新姿态:viewer 模式叠加到画面,headless 模式打印到终端。"""

    if float(data.time) - last_overlay < 0.1:
        return last_overlay
    yaw, pitch, roll = read_imu_attitude(model, data)
    line = (
        f"yaw {np.rad2deg(yaw):7.1f}°  "
        f"pitch {np.rad2deg(pitch):7.1f}°  "
        f"roll {np.rad2deg(roll):7.1f}°"
    )
    if viewer is not None:
        viewer.set_texts([(None, mujoco.mjtGridPos.mjGRID_TOPLEFT, "IMU", line)])
    else:
        print(f"[imu] t={float(data.time):6.2f}s  {line}", flush=True)
    return float(data.time)


def _validate_args(args: argparse.Namespace) -> None:
    if args.period <= 0.0 or args.amplitude < 0.0 or args.duration <= 0.0:
        raise SystemExit("高度周期和总时长必须为正数，高度幅值不得为负数")
    if args.disturbance < 0.0 or args.disturbance_period <= 0.0:
        raise SystemExit("扰动力矩不得为负数，扰动周期必须为正数")
    if not 0.0 < args.disturbance_duration < 0.5 * args.disturbance_period:
        raise SystemExit("扰动持续时间必须位于 (0, disturbance-period/2) 内")
    if args.teleop:
        # teleop 模式下不走高度正弦,幅值参数不参与高度范围校验。
        if not 0.25 <= args.height <= 0.46:
            raise SystemExit("目标高度必须位于 [0.25, 0.46] m")
    elif not 0.25 <= args.height - args.amplitude <= args.height + args.amplitude <= 0.46:
        raise SystemExit("目标高度范围必须位于 [0.25, 0.46] m")
    if args.teleop and args.headless:
        raise SystemExit("--teleop 需要 viewer,不能与 --headless 同时使用")
    if args.forward_speed < 0.0 or args.yaw_rate < 0.0 or args.decay <= 0.0:
        raise SystemExit("--forward-speed/--yaw-rate 不得为负,--decay 必须为正")


if __name__ == "__main__":
    main()

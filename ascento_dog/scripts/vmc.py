"""运行整车 VMC 演示;--teleop 用数字键 1/2/3/4 驱动,ESC 退出。"""

from __future__ import annotations

import argparse
import time
from math import isfinite, pi, sin

from ascento_dog.control import HoldTeleop, WheelSpeedGains, WheelVelocityController
from ascento_dog.kinematics import DEFAULT_GEOMETRY
from ascento_dog.simulation import (
    apply_body_disturbance,
    create_default_vmc,
    load_quadruped_model,
    set_quadruped_pose,
    step_teleop,
    step_vmc,
)
from ascento_dog.simulation.viewer import ensure_mjpython_on_macos, make_esc_exit_callback

DEFAULT_TELEOP_YAW_RATE = 4.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--height", type=float, default=0.37, help="平均目标底盘高度，m")
    parser.add_argument("--amplitude", type=float, default=0.03, help="高度正弦幅值，m")
    parser.add_argument("--period", type=float, default=8.0, help="高度变化周期，s")
    parser.add_argument("--disturbance", type=float, default=45.0, help="扰动基准力矩，N·m")
    parser.add_argument("--disturbance-period", type=float, default=4.0, help="扰动周期，s")
    parser.add_argument(
        "--disturbance-duration", type=float, default=0.15, help="单次扰动持续时间，s"
    )
    parser.add_argument(
        "--teleop", action="store_true", help="启用键盘遥控(按住动作、松开停止: 1前 2后 3左 4右)"
    )
    parser.add_argument("--forward-speed", type=float, default=1.0, help="按住时前进/后退速度, m/s")
    parser.add_argument(
        "--yaw-rate",
        type=float,
        default=DEFAULT_TELEOP_YAW_RATE,
        help="按住时转向角速度, rad/s",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _validate_args(args)
    ensure_mjpython_on_macos("ascento_dog.scripts.vmc")

    model, data = load_quadruped_model()
    set_quadruped_pose(model, data, DEFAULT_GEOMETRY.q_nominal, wheels_on_floor=True)
    controller = create_default_vmc(model)
    wheel_controller = (
        WheelVelocityController(
            WheelSpeedGains(
                kp=0.35,
                ki=0.2,
                integral_limit=3.0,
                output_limit=6.0,  # 示意值；限制急加减速时的俯仰冲击
            )
        )
        if args.teleop
        else None
    )
    teleop = HoldTeleop() if args.teleop else None
    simulation_start = float(data.time)
    _run_with_viewer(model, data, controller, args, simulation_start, teleop, wheel_controller)


def _run_with_viewer(
    model, data, controller, args, simulation_start, teleop, wheel_controller
) -> None:
    import mujoco.viewer

    from ascento_dog.simulation.teleop_viewer import NativeTeleopInput

    should_close, esc_callback = make_esc_exit_callback()
    keyboard = NativeTeleopInput(teleop, should_close) if teleop is not None else None
    try:
        with mujoco.viewer.launch_passive(
            model,
            data,
            key_callback=keyboard.key_callback if keyboard is not None else esc_callback,
            show_left_ui=True,
            show_right_ui=True,
        ) as viewer:
            if keyboard is not None:
                keyboard.viewport = lambda: viewer.viewport
            while viewer.is_running() and not should_close.is_set():
                step_start = time.monotonic()
                _step(model, data, controller, args, simulation_start, teleop, wheel_controller)
                viewer.sync()
                remaining = float(model.opt.timestep) - (time.monotonic() - step_start)
                if remaining > 0.0:
                    time.sleep(remaining)
    finally:
        if keyboard is not None:
            keyboard.stop()
            keyboard.check_error()


def _step(model, data, controller, args, simulation_start, teleop, wheel_controller) -> None:
    elapsed = float(data.time) - simulation_start
    if teleop is not None:
        command = teleop.command(
            forward_speed=args.forward_speed,
            yaw_rate=args.yaw_rate,
        )
        step_teleop(model, data, controller, wheel_controller, command, desired_height=args.height)
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


def _validate_args(args: argparse.Namespace) -> None:
    if args.period <= 0.0 or args.amplitude < 0.0:
        raise SystemExit("高度周期必须为正数，高度幅值不得为负数")
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
    if any(not isfinite(value) or value < 0.0 for value in (args.forward_speed, args.yaw_rate)):
        raise SystemExit("--forward-speed/--yaw-rate 必须为有限非负数")


if __name__ == "__main__":
    main()

"""运行整车 VMC 高度调节、三轴扰动和姿态曲线演示。"""

from __future__ import annotations

import argparse
import time
from math import pi, sin
from pathlib import Path

import numpy as np

from ascento_dog.control import PulseTeleop, WheelSpeedGains, WheelVelocityController
from ascento_dog.kinematics import DEFAULT_GEOMETRY
from ascento_dog.plotting import save_attitude_response
from ascento_dog.simulation import (
    apply_body_disturbance,
    create_default_vmc,
    load_quadruped_model,
    read_vmc_state,
    set_quadruped_pose,
    step_drive,
    step_vmc,
)
from ascento_dog.simulation.viewer import ensure_mjpython_on_macos


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
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("outputs/vmc_attitude_response"),
        help="曲线文件前缀",
    )
    parser.add_argument("--headless", action="store_true", help="不打开 MuJoCo Viewer")
    parser.add_argument("--no-show", action="store_true", help="保存曲线但不打开绘图窗口")
    parser.add_argument("--teleop", action="store_true", help="启用键盘 WASD 轮速遥杆")
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
    history: dict[str, list[float | bool]] = {
        "time": [],
        "roll": [],
        "pitch": [],
        "yaw": [],
        "disturbance": [],
    }

    if args.headless:
        while data.time - simulation_start < args.duration:
            _step_and_record(
                model, data, controller, args, simulation_start, history,
                wheel_controller, teleop,
            )
    else:
        _run_with_viewer(
            model, data, controller, args, simulation_start, history,
            wheel_controller, teleop,
        )

    csv_path, pdf_path, png_path = save_attitude_response(
        np.asarray(history["time"], dtype=float),
        np.asarray(history["roll"], dtype=float),
        np.asarray(history["pitch"], dtype=float),
        np.asarray(history["yaw"], dtype=float),
        np.asarray(history["disturbance"], dtype=bool),
        output_prefix=args.output,
        show=not args.no_show,
    )
    print(f"姿态数据: {csv_path}")
    print(f"矢量曲线: {pdf_path}")
    print(f"预览曲线: {png_path}")


def _run_with_viewer(model, data, controller, args, simulation_start, history, wheel_controller, teleop) -> None:
    import mujoco.viewer

    key_callback = None
    if teleop is not None:
        def key_callback(keycode: int) -> None:
            key = chr(keycode) if 32 <= keycode < 127 else ""
            if key and key in "WASD":
                teleop.press(key, time.monotonic())

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running() and data.time - simulation_start < args.duration:
            step_start = time.monotonic()
            _step_and_record(
                model, data, controller, args, simulation_start, history,
                wheel_controller, teleop,
            )
            viewer.sync()
            remaining = float(model.opt.timestep) - (time.monotonic() - step_start)
            if remaining > 0.0:
                time.sleep(remaining)


def _step_and_record(model, data, controller, args, simulation_start, history, wheel_controller, teleop) -> None:
    elapsed = float(data.time) - simulation_start
    if teleop is not None:
        desired_height = args.height
        command = teleop.command(
            time.monotonic(),
            forward_speed=args.forward_speed,
            yaw_rate=args.yaw_rate,
            decay=args.decay,
        )
        step_drive(model, data, controller, wheel_controller, command, desired_height=desired_height)
        active = False
    else:
        desired_height = args.height + args.amplitude * sin(2.0 * pi * elapsed / args.period)
        phase = elapsed % args.disturbance_period
        start = 0.5 * args.disturbance_period
        active = start <= phase < start + args.disturbance_duration
        magnitude = args.disturbance if active else 0.0

        # 同时激励三轴;yaw 当前没有闭环控制,因此其残余偏角会保留在曲线上。
        torque_world = (magnitude, -(2.0 / 3.0) * magnitude, magnitude / 9.0)
        apply_body_disturbance(model, data, torque_world=torque_world)
        step_vmc(model, data, controller, desired_height=desired_height)

    state = read_vmc_state(model, data)
    history["time"].append(float(data.time) - simulation_start)
    history["roll"].append(state.roll)
    history["pitch"].append(state.pitch)
    history["yaw"].append(state.yaw)
    history["disturbance"].append(active)


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

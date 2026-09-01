"""运行跨越台阶验证：整车驶向 200 mm 平台，按相位收缩/伸出四腿完成上台阶。

无参数时启动 MuJoCo 查看器（macOS 需 mjpython，自动重启），ESC 退出；
``--headless`` 适用于 CI 与测试，成功上台阶后打印结果并以返回码 0 退出。
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from ascento_dog.control import (
    CrossingCommand,
    CrossingParameters,
    LegMount,
    StepCrossingController,
    WheelObservation,
)
from ascento_dog.kinematics import DEFAULT_GEOMETRY
from ascento_dog.simulation import (
    LEG_MOUNTS,
    apply_crossing_command,
    load_step_model,
    read_hip_rates,
    read_vmc_state,
    read_wheel_centers,
    read_wheel_velocities,
    set_quadruped_pose,
)
from ascento_dog.simulation.viewer import ensure_mjpython_on_macos, make_esc_exit_callback


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="无界面运行，完成或失败后退出")
    parser.add_argument("--duration", type=float, default=30.0, help="最长仿真时长，s")
    parser.add_argument("--step-height", type=float, default=0.15, help="台阶顶面高度，m")
    parser.add_argument("--step-face-x", type=float, default=0.9, help="台阶立面前缘 x 坐标，m")
    parser.add_argument("--forward-speed", type=float, default=0.25, help="前进目标速度，m/s")
    return parser.parse_args()


def create_controller(model, args: argparse.Namespace) -> StepCrossingController:
    """按模型总质量与台阶几何构造跨越控制器。"""

    mounts = {name: LegMount(mount.position, mount.rotation) for name, mount in LEG_MOUNTS.items()}
    # 中间过程车体保持水平：低站姿高度 = 台阶顶面 + 轮半径 + 最短腿下探量；
    # 台上标称站姿高度 = 台阶顶面 + 轮半径 + 标称姿态轮心下探量。
    d_min = -DEFAULT_GEOMETRY.forward(DEFAULT_GEOMETRY.q_max).e[1]
    nominal_drop = -DEFAULT_GEOMETRY.forward(DEFAULT_GEOMETRY.q_nominal).e[1]
    parameters = CrossingParameters(
        step_height=args.step_height,
        step_face_x=args.step_face_x,
        stance_height=args.step_height + 0.065 + d_min,
        extend_height=args.step_height + 0.065 + nominal_drop,
        forward_speed=args.forward_speed,
    )
    return StepCrossingController(
        mass=float(np.sum(model.body_mass)),
        mounts=mounts,
        parameters=parameters,
        geometry=DEFAULT_GEOMETRY,
    )


def step_once(model, data, controller: StepCrossingController) -> CrossingCommand:
    """读取状态、计算并写入跨越指令，推进 MuJoCo 一步。"""

    import mujoco

    state = read_vmc_state(model, data)
    wheels = {
        name: WheelObservation(x=float(position[0]), z=float(position[2]))
        for name, position in read_wheel_centers(model, data).items()
    }
    command = controller.update(
        state,
        wheels,
        wheel_velocities=read_wheel_velocities(model, data),
        leg_rates=read_hip_rates(model, data),
        dt=float(model.opt.timestep),
    )
    apply_crossing_command(model, data, command)
    mujoco.mj_step(model, data)
    return command


def _validate_args(args: argparse.Namespace) -> None:
    if args.duration <= 0.0:
        raise SystemExit("--duration 必须为正数")
    if args.step_height <= 0.0:
        raise SystemExit("--step-height 必须为正数")
    if args.forward_speed <= 0.0:
        raise SystemExit("--forward-speed 必须为正数")


def _summary(model, data) -> str:
    """返回整车最终状态的简要描述，用于人工核对。"""

    centers = read_wheel_centers(model, data)
    state = read_vmc_state(model, data)
    return (
        "轮心高度 (m): "
        + ", ".join(f"{name}={float(z[2]):.3f}" for name, z in centers.items())
        + f"; 底盘高度={state.height:.3f} m"
        + f"; 俯仰={np.degrees(state.pitch):.1f} deg"
        + f"; 横滚={np.degrees(state.roll):.1f} deg"
    )


def run_headless(model, data, controller: StepCrossingController, args: argparse.Namespace) -> int:
    """无界面运行至完成/失败/超时，返回进程退出码。"""

    start = float(data.time)
    while float(data.time) - start < args.duration:
        command = step_once(model, data, controller)
        if command.failed:
            print(f"[cross-step] 失败: {command.failure_reason}")
            return 1
        if command.done:
            # 完成后再保持 1 s，验证姿态稳定。
            hold_until = float(data.time) + 1.0
            while float(data.time) < hold_until:
                step_once(model, data, controller)
            print(f"[cross-step] 成功上台阶，用时 {float(data.time) - start - 1.0:.2f} s")
            print(f"[cross-step] {_summary(model, data)}")
            return 0
    print(f"[cross-step] 超时: {controller.phase.value}")
    return 1


def main() -> None:
    args = parse_args()
    _validate_args(args)
    if not args.headless:
        ensure_mjpython_on_macos("ascento_dog.scripts.cross_step")

    model, data = load_step_model()
    set_quadruped_pose(model, data, DEFAULT_GEOMETRY.q_nominal, wheels_on_floor=True)
    controller = create_controller(model, args)

    if args.headless:
        raise SystemExit(run_headless(model, data, controller, args))

    _run_with_viewer(model, data, controller, args)


def _run_with_viewer(
    model, data, controller: StepCrossingController, args: argparse.Namespace
) -> None:
    import mujoco.viewer

    should_close, esc_callback = make_esc_exit_callback()

    def key_callback(keycode: int) -> None:
        esc_callback(keycode)

    with mujoco.viewer.launch_passive(
        model, data, key_callback=key_callback, show_left_ui=True, show_right_ui=True
    ) as viewer:
        while viewer.is_running() and not should_close.is_set():
            step_start = time.monotonic()
            command = step_once(model, data, controller)
            viewer.sync()
            if command.failed:
                print(f"[cross-step] 失败: {command.failure_reason}")
                break
            remaining = float(model.opt.timestep) - (time.monotonic() - step_start)
            if remaining > 0.0:
                time.sleep(remaining)
        if command.done:
            print(f"[cross-step] 成功: {_summary(model, data)}")


if __name__ == "__main__":
    main()

"""显示单腿悬空模型并扫描解析运动学工作区间。"""

from __future__ import annotations

import argparse
import time
from math import pi, sin

from ascento_dog.kinematics import DEFAULT_GEOMETRY
from ascento_dog.simulation import load_single_leg_model, set_kinematic_pose
from ascento_dog.simulation.viewer import ensure_mjpython_on_macos, make_esc_exit_callback


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--period", type=float, default=4.0, help="往复运动周期，s")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.period <= 0.0:
        raise SystemExit("--period 必须为正数")
    ensure_mjpython_on_macos("ascento_dog.scripts.leg_kinematics")

    import mujoco.viewer

    model, data = load_single_leg_model()
    midpoint = 0.5 * (DEFAULT_GEOMETRY.q_min + DEFAULT_GEOMETRY.q_max)
    amplitude = 0.5 * (DEFAULT_GEOMETRY.q_max - DEFAULT_GEOMETRY.q_min)
    start = time.monotonic()
    should_close, key_callback = make_esc_exit_callback()
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running() and not should_close.is_set():
            elapsed = time.monotonic() - start
            q = midpoint + amplitude * sin(2.0 * pi * elapsed / args.period)
            set_kinematic_pose(model, data, q)
            viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    main()

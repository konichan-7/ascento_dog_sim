"""Animate the closed-loop single leg through its analytical working range."""

from __future__ import annotations

import argparse
import time
from math import sin

from ascento_dog.kinematics import DEFAULT_GEOMETRY
from ascento_dog.simulation import load_single_leg_model, set_kinematic_pose


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--period", type=float, default=4.0, help="sweep period in seconds")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.period <= 0.0:
        raise SystemExit("--period must be positive")

    import mujoco
    import mujoco.viewer

    model, data = load_single_leg_model()
    midpoint = 0.5 * (DEFAULT_GEOMETRY.q_min + DEFAULT_GEOMETRY.q_max)
    amplitude = 0.5 * (DEFAULT_GEOMETRY.q_max - DEFAULT_GEOMETRY.q_min)
    start = time.monotonic()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            elapsed = time.monotonic() - start
            q = midpoint + amplitude * sin(2.0 * 3.141592653589793 * elapsed / args.period)
            set_kinematic_pose(model, data, q)
            viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    main()

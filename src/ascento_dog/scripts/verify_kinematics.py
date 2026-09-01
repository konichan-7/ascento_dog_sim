"""Headless analytical and MuJoCo kinematics verification command."""

from __future__ import annotations

import argparse
from math import degrees

import numpy as np

from ascento_dog.kinematics import DEFAULT_GEOMETRY
from ascento_dog.simulation import verify_mujoco_kinematics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=101, help="number of hip angles")
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1.0e-8,
        help="maximum accepted metric and angular error",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.samples < 2:
        raise SystemExit("--samples must be at least 2")
    if args.tolerance <= 0.0:
        raise SystemExit("--tolerance must be positive")

    max_bar_error = 0.0
    max_angle_error = 0.0
    wheel_x: list[float] = []
    wheel_z: list[float] = []
    for q in np.linspace(DEFAULT_GEOMETRY.q_min, DEFAULT_GEOMETRY.q_max, args.samples):
        pose = DEFAULT_GEOMETRY.forward(float(q))
        max_bar_error = max(
            max_bar_error,
            max(abs(value) for value in DEFAULT_GEOMETRY.bar_residuals(pose).values()),
        )
        recovered = DEFAULT_GEOMETRY.inverse(pose.e)
        max_angle_error = max(max_angle_error, abs(recovered - q))
        wheel_x.append(float(pose.e[0]))
        wheel_z.append(float(pose.e[1]))

    mujoco_report = verify_mujoco_kinematics(samples=args.samples)
    print(f"samples: {args.samples}")
    print(
        "hip range: "
        f"[{degrees(DEFAULT_GEOMETRY.q_min):.3f}, "
        f"{degrees(DEFAULT_GEOMETRY.q_max):.3f}] deg"
    )
    print(f"wheel z range: [{min(wheel_z):.6f}, {max(wheel_z):.6f}] m")
    print(f"wheel x excursion: {max(wheel_x) - min(wheel_x):.6e} m")
    print(f"max bar residual: {max_bar_error:.6e} m")
    print(f"max IK(FK(q)) error: {max_angle_error:.6e} rad")
    print(f"max MuJoCo point error: {mujoco_report.max_point_error_m:.6e} m")
    print(f"max MuJoCo loop error: {mujoco_report.max_loop_error_m:.6e} m")

    worst_error = max(
        max_bar_error,
        max_angle_error,
        mujoco_report.max_point_error_m,
        mujoco_report.max_loop_error_m,
    )
    if worst_error > args.tolerance:
        raise SystemExit(
            f"verification failed: {worst_error:.6e} exceeds {args.tolerance:.6e}"
        )


if __name__ == "__main__":
    main()

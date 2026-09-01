"""Ascento-style robot simulation foundations."""

from pathlib import Path

from ascento_dog.kinematics.four_bar import DEFAULT_GEOMETRY, FourBarGeometry

# 仿真代码物理位于 mujoco/simulation/ 下。mujoco/ 不能作为 Python 包
# (会遮蔽同名的 mujoco 绑定库),因此把 mujoco/ 加入本包的 __path__,
# 使 ascento_dog.simulation.* 仍可解析到 mujoco/simulation/*.py。
__path__.append(str(Path(__file__).resolve().parent.parent / "mujoco"))

__all__ = ["DEFAULT_GEOMETRY", "FourBarGeometry"]

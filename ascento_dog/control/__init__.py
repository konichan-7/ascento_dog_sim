"""Control algorithms independent of the MuJoCo simulation adapter."""

from ascento_dog.control.vmc import (
    AttitudeGains,
    HeightPID,
    LegMount,
    PIDGains,
    QuadrupedVMC,
    VMCCommand,
    VMCState,
    allocate_vertical_forces,
    vertical_force_allocation_matrix,
)
from ascento_dog.control.wheel_speed import (
    DriveCommand,
    PulseTeleop,
    WheelSpeedGains,
    WheelVelocityController,
    wheel_speed_targets,
)

__all__ = [
    "AttitudeGains",
    "HeightPID",
    "LegMount",
    "PIDGains",
    "QuadrupedVMC",
    "VMCCommand",
    "VMCState",
    "allocate_vertical_forces",
    "vertical_force_allocation_matrix",
    "DriveCommand",
    "PulseTeleop",
    "WheelSpeedGains",
    "WheelVelocityController",
    "wheel_speed_targets",
]

"""
TM5 coordinate representation.

The project robot-side coordinate convention matches
the TM5-700 MoveIt base frame.

X = left / right
Y = forward / depth
Z = height

All inputs/outputs are millimetres.
"""

from dataclasses import dataclass


@dataclass
class TM5Coordinate:
    x_mm: float
    y_mm: float
    z_mm: float


def fyp_world_to_tm5(world):
    """
    Convert a WorldCoordinate into a TM5Coordinate.

    WorldCoordinate is already expressed using the
    TM5 / MoveIt base-axis convention, so no axis
    permutation is required here.
    """

    if world is None:
        return None

    return TM5Coordinate(
        x_mm=float(world.x_mm),
        y_mm=float(world.y_mm),
        z_mm=float(world.z_mm)
    )

from dataclasses import dataclass
from .shelf_location import ShelfLocation


@dataclass(frozen=True)
class RobotHome:
    location: ShelfLocation
    joint_positions: tuple[float, ...]


ROBOT_HOMES = {
    ShelfLocation.BOTTOM: RobotHome(
        location=ShelfLocation.BOTTOM,
        joint_positions=(
            -0.00005599,
            0.69210843,
            2.42650381,
            -3.13254270,
            1.57113908,
            0.00004000,
        ),
    ),

    ShelfLocation.TOP: RobotHome(
        location=ShelfLocation.TOP,
        joint_positions=(
            -0.00005599,
            -0.25513466,
            2.08663958,
            -1.84544959,
            1.57014998,
            0.00003002,
        ),
    ),
}


def get_robot_home(location):
    return ROBOT_HOMES[location]

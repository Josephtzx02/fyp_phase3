from dataclasses import dataclass
from enum import Enum


class ShelfLocation(Enum):
    BOTTOM = "bottom"
    TOP = "top"


@dataclass(frozen=True)
class ShelfConfig:
    location: ShelfLocation
    z_mm: float


SHELF_CONFIGS = {
    ShelfLocation.BOTTOM: ShelfConfig(
        location=ShelfLocation.BOTTOM,
        z_mm=200.0,
    ),

    ShelfLocation.TOP: ShelfConfig(
        location=ShelfLocation.TOP,
        z_mm=616.0,
    ),
}


def get_shelf_config(location):
    return SHELF_CONFIGS[location]

from dataclasses import dataclass
from enum import Enum

from .shelf_location import ShelfLocation


class MissionType(Enum):
    RETRIEVE_ONLY = "retrieve_only"
    TRANSFER_UP = "transfer_up"
    TRANSFER_DOWN = "transfer_down"


@dataclass(frozen=True)
class Mission:
    source: ShelfLocation
    destination: ShelfLocation

    @property
    def mission_type(self):
        if self.source == self.destination:
            return MissionType.RETRIEVE_ONLY

        if (
            self.source == ShelfLocation.BOTTOM
            and self.destination == ShelfLocation.TOP
        ):
            return MissionType.TRANSFER_UP

        if (
            self.source == ShelfLocation.TOP
            and self.destination == ShelfLocation.BOTTOM
        ):
            return MissionType.TRANSFER_DOWN

        raise ValueError(
            f"Unsupported mission: "
            f"{self.source.value} -> {self.destination.value}"
        )

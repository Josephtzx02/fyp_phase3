"""
This module converts perception output into a unified robotic grasp target.

Everything downstream should use GraspTarget.
"""

from dataclasses import dataclass
from .world_transform import camera_to_world
from .shelf_location import ShelfLocation


@dataclass
class GraspTarget:
    """
    Unified grasp target using the TM5 / MoveIt base convention.

    X = left / right
    Y = forward / depth
    Z = height
    """
    shelf_location: ShelfLocation

    # Physical Book Dimensions
    book_x_mm: float
    book_y_mm: float
    book_z_mm: float

    width_mm: float
    height_mm: float
    thickness_mm: float

    angle_deg: float

    # Grasp
    grasp_x_mm: float
    grasp_y_mm: float
    grasp_z_mm: float

    # Approach
    approach_x_mm: float
    approach_y_mm: float
    approach_z_mm: float

    # Original Camera Coordinate
    camera_x_mm: float
    camera_y_mm: float
    camera_z_mm: float

    @property
    def book_position(self):
        return (
            self.book_x_mm,
            self.book_y_mm,
            self.book_z_mm
        )

    @property
    def grasp_position(self):
        return (
            self.grasp_x_mm,
            self.grasp_y_mm,
            self.grasp_z_mm
        )

    @property
    def approach_position(self):
        return (
            self.approach_x_mm,
            self.approach_y_mm,
            self.approach_z_mm
        )

    @property
    def camera_position(self):
        return (
            self.camera_x_mm,
            self.camera_y_mm,
            self.camera_z_mm
        )


def create_grasp_target(det, shelf_location):
    """
    Convert one perception detection into one grasp target.

    Coordinates returned by camera_to_world() are already
    expressed using the TM5 / MoveIt base convention:

        X = left / right
        Y = forward / depth
        Z = height
    """

    camera_coord = det.get("camera_coord")

    if camera_coord is None:
        return None

    # Physical dimensions
    width = det.get("pred_width_mm")
    height = det.get("height_mm")
    thickness = det.get("thickness_mm")

    # Fallback values
    if width is None:
        width = 150.0

    if height is None:
        height = 220.0

    if thickness is None:
        thickness = 30.0

    width = float(width)
    height = float(height)
    thickness = float(thickness)
    
    world = camera_to_world(camera_coord, thickness, shelf_location)

    if world is None:
        return None

    # TM5 / MoveIt base coordinates
    book_x = float(world.x_mm)
    book_y = float(world.y_mm)
    book_z = float(world.z_mm)

    # Grasp at detected book centre
    grasp_x = book_x
    grasp_y = book_y
    grasp_z = book_z

    # Approach from further away along robot X/depth axis. Previous value: 100.0
    APPROACH_OFFSET = 177.0

    approach_x = grasp_x - APPROACH_OFFSET
    approach_y = grasp_y
    approach_z = grasp_z

    return GraspTarget(

        # Book
        book_x_mm=book_x,
        book_y_mm=book_y,
        book_z_mm=book_z,

        width_mm=width,
        height_mm=height,
        thickness_mm=thickness,
        
        # Shelf Location
        shelf_location=shelf_location,

        angle_deg=float(det["angle"]),

        # Grasp
        grasp_x_mm=grasp_x,
        grasp_y_mm=grasp_y,
        grasp_z_mm=grasp_z,

        # Approach
        approach_x_mm=approach_x,
        approach_y_mm=approach_y,
        approach_z_mm=approach_z,

        # Camera Coordinate
        camera_x_mm=float(camera_coord.x_mm),
        camera_y_mm=float(camera_coord.y_mm),
        camera_z_mm=float(camera_coord.z_mm)
    )

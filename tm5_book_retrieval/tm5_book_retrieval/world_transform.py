"""
world_transform.py

Convert Camera Coordinates
into TM5 / MoveIt Robot Base Coordinates.

Robot / MoveIt Base Coordinate System

             +Z
             |
             | Height
             |
             O----------> +X
            /
           /
         +Y
       Forward / Depth

X = Left / Right
Y = Forward / Depth
Z = Height

All distances are millimetres.
"""

from dataclasses import dataclass


@dataclass
class WorldCoordinate:
    x_mm: float
    y_mm: float
    z_mm: float


def camera_to_world(camera_coord, thickness, shelf_location):
    """
    Convert RealSense camera coordinates into TM5 base coordinates.

    TM5 base:
        X = left/right
        Y = forward/depth
        Z = height above TM5 base

    Camera:
        X = left/right
        Y = down
        Z = forward
    """

    if camera_coord is None:
        return None

    ROBOT_REF_X_MM = 285.0 # Actual Reset X is 330.0, Previous was 300.0
    ROBOT_REF_Y_MM = -122.0
    
    if shelf_location.value == "bottom":
        ROBOT_REF_Z_MM = 195.0  # Previous Z_mm value: 258.0
    elif shelf_location.value == "top":
        ROBOT_REF_Z_MM = 490.0
    else:
        raise ValueError(f"Unknown shelf location: {shelf_location}")
    
    world_x = ROBOT_REF_X_MM + camera_coord.z_mm
    Y_CORRECTION_MM = 25.0
    y_arm_thickness = 5.5
    # Previous Y_correction: 33.0, 6.5
    world_y = ROBOT_REF_Y_MM - camera_coord.x_mm + Y_CORRECTION_MM + y_arm_thickness + thickness/2.0
    Z_CORRECTION_MM = 68.0 
    world_z = ROBOT_REF_Z_MM - camera_coord.y_mm + Z_CORRECTION_MM

    return WorldCoordinate(
        x_mm=world_x,
        y_mm=world_y,
        z_mm=world_z
    )

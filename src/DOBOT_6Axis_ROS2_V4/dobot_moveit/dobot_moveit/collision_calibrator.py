"""
Collision calibration geometry helpers.

Pure Python functions (no ROS2 / numpy dependencies) that convert
millimetre-space calibration measurements into MoveIt-compatible
box parameters (center in metres, dimensions in metres).

These functions are consumed by the CollisionCalibrator node (Task 2)
and tested standalone by scripts/test_collision_geometry.py.
"""

import math


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def corners_to_box(p1_mm, p2_mm, height_mm):
    """Two diagonal top-surface corners + object height -> MoveIt box params.

    Args:
        p1_mm:     [x, y, z] first corner on the top surface (mm).
        p2_mm:     [x, y, z] second diagonal corner on the top surface (mm).
        height_mm: Positive height of the box (mm).

    Returns:
        (center_m, dimensions_m) where both are [x, y, z] lists in metres.
        center XY  = midpoint of p1, p2.
        center Z   = avg(p1.z, p2.z) - height_mm / 2  (half-height below surface).
        dimensions = [|x2-x1|, |y2-y1|, height] all converted to metres.

    Raises:
        ValueError: if any resulting dimension is < 50 mm.
    """
    dx = abs(p2_mm[0] - p1_mm[0])
    dy = abs(p2_mm[1] - p1_mm[1])
    dz = height_mm

    if dx < 50.0:
        raise ValueError(f"Width {dx:.1f} mm is below 50 mm minimum")
    if dy < 50.0:
        raise ValueError(f"Depth {dy:.1f} mm is below 50 mm minimum")
    if dz < 50.0:
        raise ValueError(f"Height {dz:.1f} mm is below 50 mm minimum")

    cx = (p1_mm[0] + p2_mm[0]) / 2.0
    cy = (p1_mm[1] + p2_mm[1]) / 2.0
    z_surface = (p1_mm[2] + p2_mm[2]) / 2.0
    cz = z_surface - height_mm / 2.0

    center_m = [cx / 1000.0, cy / 1000.0, cz / 1000.0]
    dimensions_m = [dx / 1000.0, dy / 1000.0, dz / 1000.0]
    return center_m, dimensions_m


def floor_plane(z_mm):
    """One floor Z measurement -> large collision plane.

    Returns:
        (center_m, dimensions_m) — a 6 m x 6 m x 0.01 m slab whose top
        surface sits at z_mm.
    """
    center_m = [0.0, 0.0, z_mm / 1000.0 - 0.005]
    dimensions_m = [6.0, 6.0, 0.01]
    return center_m, dimensions_m


def ceiling_plane(z_mm):
    """One ceiling Z measurement -> large collision plane.

    Returns:
        (center_m, dimensions_m) — a 6 m x 6 m x 0.01 m slab whose bottom
        surface sits at z_mm.
    """
    center_m = [0.0, 0.0, z_mm / 1000.0 + 0.005]
    dimensions_m = [6.0, 6.0, 0.01]
    return center_m, dimensions_m


def pole_box(bottom_mm, width_mm, ceiling_z_mm):
    """Camera pole: bottom point + width + ceiling Z -> vertical box.

    Args:
        bottom_mm:    [x, y, z] bottom centre of the pole (mm).
        width_mm:     Cross-section width (mm), used for both X and Y.
        ceiling_z_mm: Z coordinate of the ceiling (mm).

    Returns:
        (center_m, dimensions_m) where dimensions = [w, w, h] in metres.

    Raises:
        ValueError: if width_mm < 10 or computed height < 50 mm.
    """
    if width_mm < 10.0:
        raise ValueError(f"Pole width {width_mm:.1f} mm is below 10 mm minimum")

    h = ceiling_z_mm - bottom_mm[2]
    if h < 50.0:
        raise ValueError(f"Pole height {h:.1f} mm is below 50 mm minimum")

    cz = (bottom_mm[2] + ceiling_z_mm) / 2.0
    center_m = [bottom_mm[0] / 1000.0, bottom_mm[1] / 1000.0, cz / 1000.0]
    dimensions_m = [width_mm / 1000.0, width_mm / 1000.0, h / 1000.0]
    return center_m, dimensions_m


def wall_box(p1_mm, p2_mm, ceiling_z_mm, floor_z_mm, thickness_mm=100.0):
    """Two base points -> wall from floor to ceiling.

    The wall runs along the line from p1 to p2.  Thickness is perpendicular
    to that line.

    Args:
        p1_mm:         [x, y, z] first base point (mm).
        p2_mm:         [x, y, z] second base point (mm).
        ceiling_z_mm:  Z of the ceiling (mm).
        floor_z_mm:    Z of the floor (mm).
        thickness_mm:  Wall thickness (mm, default 100).

    Returns:
        (center_m, dimensions_m, rotation_rad)
        - center_m      = midpoint at half-height (metres).
        - dimensions_m   = [length, thickness, height] in metres.
        - rotation_rad   = math.atan2(dy, dx) — rotation around Z axis.

    Raises:
        ValueError: if wall length (distance p1 -> p2) < 100 mm.
    """
    dx = p2_mm[0] - p1_mm[0]
    dy = p2_mm[1] - p1_mm[1]
    length = math.sqrt(dx * dx + dy * dy)

    if length < 100.0:
        raise ValueError(f"Wall length {length:.1f} mm is below 100 mm minimum")

    height = ceiling_z_mm - floor_z_mm
    cx = (p1_mm[0] + p2_mm[0]) / 2.0
    cy = (p1_mm[1] + p2_mm[1]) / 2.0
    cz = (floor_z_mm + ceiling_z_mm) / 2.0

    center_m = [cx / 1000.0, cy / 1000.0, cz / 1000.0]
    dimensions_m = [length / 1000.0, thickness_mm / 1000.0, height / 1000.0]
    rotation_rad = math.atan2(dy, dx)

    return center_m, dimensions_m, rotation_rad


def derive_place_position(p1_mm, p2_mm, height_mm, offset_mm=20.0):
    """Derive place_position from cinta (conveyor belt) corners.

    Args:
        p1_mm:     [x, y, z] first corner on the top surface (mm).
        p2_mm:     [x, y, z] second diagonal corner (mm).
        height_mm: Object height used during calibration (mm) — not used
                   directly; z_top is taken from the corner Z values.
        offset_mm: Extra clearance above the surface (mm, default 20).

    Returns:
        [center_x_m, center_y_m, place_z_m] in metres.
        z_top_mm = avg(p1.z, p2.z)  (the surface of the cinta).
        place_z  = (z_top_mm + offset_mm) / 1000.
    """
    cx = (p1_mm[0] + p2_mm[0]) / 2.0
    cy = (p1_mm[1] + p2_mm[1]) / 2.0
    z_top = (p1_mm[2] + p2_mm[2]) / 2.0

    return [cx / 1000.0, cy / 1000.0, (z_top + offset_mm) / 1000.0]


def validate_config(config):
    """Validate a complete collision config dictionary.

    Checks:
        - 'objects' key exists
        - All 7 expected object keys are present:
          mesa, cinta, suelo, techo, poste_camara, pared_izq, pared_der
        - ceiling_z > floor_z
        - place_position_m has exactly 3 elements

    Args:
        config: dict — the collision calibration config.

    Returns:
        (is_valid, errors) where is_valid is bool and errors is list[str].
    """
    errors = []

    expected_objects = [
        'suelo',
        'techo',
        'mesa_trabajo',
        'pallet_jugos',
        'cinta_transportadora',
        'camara_soporte',
        'pared',
    ]

    if 'objects' not in config:
        errors.append("Missing 'objects' key in config")
    else:
        for key in expected_objects:
            if key not in config['objects']:
                errors.append(f"Missing object key: '{key}'")

    # ceiling_z > floor_z
    objects = config.get('objects', {})
    techo = objects.get('techo', {})
    suelo = objects.get('suelo', {})
    ceiling_z = techo.get('z_mm')
    floor_z = suelo.get('z_mm')
    if ceiling_z is not None and floor_z is not None:
        if ceiling_z <= floor_z:
            errors.append(
                f"ceiling_z ({ceiling_z}) must be greater than floor_z ({floor_z})"
            )

    # place_position_m
    place = config.get('place_position_m')
    if place is not None:
        if not isinstance(place, (list, tuple)) or len(place) != 3:
            errors.append(
                f"place_position_m must have 3 elements, got {len(place) if isinstance(place, (list, tuple)) else type(place).__name__}"
            )

    return (len(errors) == 0, errors)

#!/usr/bin/env python3
"""
Standalone unit tests for collision_calibrator geometry helpers.

No ROS2 or numpy dependencies — runs with plain Python 3.

Usage:
    cd dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_moveit
    python3 scripts/test_collision_geometry.py
"""

import math
import sys
import os

# Ensure we can import from the package directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'dobot_moveit'))

from collision_calibrator import (
    corners_to_box,
    floor_plane,
    ceiling_plane,
    pole_box,
    wall_box,
    derive_place_position,
    validate_config,
)

# ---------------------------------------------------------------------------
# Colours
# ---------------------------------------------------------------------------
GREEN = '\033[92m'
RED = '\033[91m'
CYAN = '\033[96m'
RESET = '\033[0m'

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
TOL = 1e-6

results = []  # list of (name, passed, detail)


def close(a, b):
    """Float comparison within tolerance."""
    return abs(a - b) < TOL


def close_list(a, b):
    """Element-wise float comparison for two lists."""
    if len(a) != len(b):
        return False
    return all(close(x, y) for x, y in zip(a, b))


def record(name, passed, detail=''):
    results.append((name, passed, detail))
    icon = f'{GREEN}\u2705{RESET}' if passed else f'{RED}\u274c{RESET}'
    print(f'  {icon}  {name}: {detail}')


# ===========================================================================
# Tests
# ===========================================================================

def test_corners_to_box_basic():
    """Test 1: corners_to_box with realistic workspace coords."""
    center, dims = corners_to_box([300, 800, -500], [700, 1200, -500], 200)
    expected_center = [0.5, 1.0, -0.6]
    expected_dims = [0.4, 0.4, 0.2]
    ok = close_list(center, expected_center) and close_list(dims, expected_dims)
    detail = f'center={center}, dims={dims}'
    if not ok:
        detail += f' (expected center={expected_center}, dims={expected_dims})'
    record('corners_to_box_basic', ok, detail)


def test_corners_to_box_zero_dims():
    """Test 2: corners_to_box with identical points -> ValueError (dim < 50mm)."""
    try:
        corners_to_box([500, 500, -500], [500, 500, -500], 200)
        record('corners_to_box_zero_dims', False, 'Expected ValueError not raised')
    except ValueError as e:
        record('corners_to_box_zero_dims', True, f'ValueError: {e}')


def test_corners_to_box_narrow():
    """Test 3: corners_to_box with width < 50mm -> ValueError."""
    try:
        corners_to_box([300, 800, -500], [305, 1200, -500], 200)
        record('corners_to_box_narrow', False, 'Expected ValueError not raised')
    except ValueError as e:
        record('corners_to_box_narrow', True, f'ValueError: {e}')


def test_floor_plane():
    """Test 4: floor_plane at Z=-510mm."""
    center, dims = floor_plane(-510)
    expected_center = [0.0, 0.0, -0.515]
    expected_dims = [6.0, 6.0, 0.01]
    ok = close_list(center, expected_center) and close_list(dims, expected_dims)
    detail = f'center={center}, dims={dims}'
    if not ok:
        detail += f' (expected center={expected_center}, dims={expected_dims})'
    record('floor_plane', ok, detail)


def test_ceiling_plane():
    """Test 5: ceiling_plane at Z=800mm."""
    center, dims = ceiling_plane(800)
    expected_center = [0.0, 0.0, 0.805]
    expected_dims = [6.0, 6.0, 0.01]
    ok = close_list(center, expected_center) and close_list(dims, expected_dims)
    detail = f'center={center}, dims={dims}'
    if not ok:
        detail += f' (expected center={expected_center}, dims={expected_dims})'
    record('ceiling_plane', ok, detail)


def test_pole_box():
    """Test 6: pole_box with bottom=[500,1000,200], width=80, ceiling=800."""
    center, dims = pole_box([500, 1000, 200], 80, 800)
    expected_center = [0.5, 1.0, 0.5]
    expected_dims = [0.08, 0.08, 0.6]
    ok = close_list(center, expected_center) and close_list(dims, expected_dims)
    detail = f'center={center}, dims={dims}'
    if not ok:
        detail += f' (expected center={expected_center}, dims={expected_dims})'
    record('pole_box', ok, detail)


def test_derive_place_position():
    """Test 7: derive_place_position from cinta corners."""
    result = derive_place_position([400, -500, -300], [600, -300, -300], 100, offset_mm=20)
    expected = [0.5, -0.4, -0.28]
    ok = close_list(result, expected)
    detail = f'result={result}'
    if not ok:
        detail += f' (expected {expected})'
    record('derive_place_position', ok, detail)


def test_mm_to_m_conversion():
    """Test 8: mm->m conversion via corners_to_box / floor_plane."""
    # 500mm -> 0.5m
    c1, _ = corners_to_box([0, 0, 500], [1000, 1000, 500], 200)
    ok1 = close(500.0 / 1000.0, 0.5)

    # -505mm -> -0.505m
    c2, _ = floor_plane(-505)
    ok2 = close(-505.0 / 1000.0 - 0.005, c2[2])

    # 1000mm -> 1.0m
    c3, _ = ceiling_plane(1000)
    ok3 = close(1000.0 / 1000.0 + 0.005, c3[2])

    ok = ok1 and ok2 and ok3
    detail = f'500mm->0.5m:{ok1}, -505mm->-0.51m:{ok2}, 1000mm->1.005m:{ok3}'
    record('mm_to_m_conversion', ok, detail)


def test_validate_config_valid():
    """Test 9: validate_config with a complete valid config."""
    config = {
        'objects': {
            'mesa':          {'corners': [[300, 800, -500], [700, 1200, -500]], 'height_mm': 200},
            'cinta':         {'corners': [[400, -500, -300], [600, -300, -300]], 'height_mm': 100},
            'suelo':         {'z_mm': -510},
            'techo':         {'z_mm': 800},
            'poste_camara':  {'bottom': [500, 1000, 200], 'width_mm': 80},
            'pared_izq':     {'points': [[0, 0, 0], [1000, 0, 0]]},
            'pared_der':     {'points': [[0, 1500, 0], [1000, 1500, 0]]},
        },
        'place_position_m': [0.5, -0.4, -0.28],
    }
    is_valid, errors = validate_config(config)
    ok = is_valid and len(errors) == 0
    detail = f'is_valid={is_valid}, errors={errors}'
    record('validate_config_valid', ok, detail)


def test_validate_config_ceiling_below_floor():
    """Test 10: validate_config with ceiling_z < floor_z -> invalid."""
    config = {
        'objects': {
            'mesa':          {},
            'cinta':         {},
            'suelo':         {'z_mm': 800},
            'techo':         {'z_mm': -510},
            'poste_camara':  {},
            'pared_izq':     {},
            'pared_der':     {},
        },
        'place_position_m': [0.5, -0.4, -0.28],
    }
    is_valid, errors = validate_config(config)
    ok = not is_valid and len(errors) > 0
    detail = f'is_valid={is_valid}, errors={errors}'
    record('validate_config_ceiling_below_floor', ok, detail)


def test_wall_box_basic():
    """Test 11 (bonus): wall_box with axis-aligned wall."""
    center, dims, rot = wall_box([0, 0, 0], [2000, 0, 0], 800, -510, thickness_mm=100)
    expected_center = [1.0, 0.0, 0.145]
    expected_dims = [2.0, 0.1, 1310.0 / 1000.0]
    expected_rot = 0.0
    ok = (close_list(center, expected_center)
          and close_list(dims, expected_dims)
          and close(rot, expected_rot))
    detail = f'center={center}, dims={dims}, rot={rot:.4f}'
    if not ok:
        detail += f' (expected center={expected_center}, dims={expected_dims}, rot={expected_rot})'
    record('wall_box_basic', ok, detail)


def test_wall_box_angled():
    """Test 12 (bonus): wall_box at 45 degrees."""
    center, dims, rot = wall_box([0, 0, 0], [1000, 1000, 0], 500, -500)
    expected_length = math.sqrt(1000**2 + 1000**2) / 1000.0
    expected_rot = math.atan2(1000, 1000)  # pi/4
    ok = close(dims[0], expected_length) and close(rot, expected_rot)
    detail = f'length={dims[0]:.4f} (exp {expected_length:.4f}), rot={rot:.4f} (exp {expected_rot:.4f})'
    record('wall_box_angled', ok, detail)


def test_wall_box_too_short():
    """Test 13 (bonus): wall_box with length < 100mm -> ValueError."""
    try:
        wall_box([0, 0, 0], [50, 0, 0], 800, -510)
        record('wall_box_too_short', False, 'Expected ValueError not raised')
    except ValueError as e:
        record('wall_box_too_short', True, f'ValueError: {e}')


def test_pole_box_narrow():
    """Test 14 (bonus): pole_box with width < 10mm -> ValueError."""
    try:
        pole_box([500, 1000, 200], 5, 800)
        record('pole_box_narrow', False, 'Expected ValueError not raised')
    except ValueError as e:
        record('pole_box_narrow', True, f'ValueError: {e}')


# ===========================================================================
# Runner
# ===========================================================================

def main():
    print(f'\n{CYAN}=== Collision Geometry Unit Tests ==={RESET}\n')

    tests = [
        test_corners_to_box_basic,        # 1
        test_corners_to_box_zero_dims,    # 2
        test_corners_to_box_narrow,       # 3
        test_floor_plane,                 # 4
        test_ceiling_plane,               # 5
        test_pole_box,                    # 6
        test_derive_place_position,       # 7
        test_mm_to_m_conversion,          # 8
        test_validate_config_valid,       # 9
        test_validate_config_ceiling_below_floor,  # 10
        test_wall_box_basic,              # 11
        test_wall_box_angled,             # 12
        test_wall_box_too_short,          # 13
        test_pole_box_narrow,             # 14
    ]

    for test_fn in tests:
        test_fn()

    passed = sum(1 for _, p, _ in results if p)
    total = len(results)

    print(f'\n{CYAN}=== [{passed}/{total}] tests passed ==={RESET}\n')

    if passed < total:
        print(f'{RED}FAILED tests:{RESET}')
        for name, p, detail in results:
            if not p:
                print(f'  {RED}\u274c{RESET}  {name}: {detail}')

    sys.exit(0 if passed == total else 1)


if __name__ == '__main__':
    main()

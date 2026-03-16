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
    """Validate a collision config dictionary.

    Checks:
        - 'objects' key exists
        - Exactly ONE object has type='floor'
        - Exactly ONE object has type='ceiling'
        - ceiling_z > floor_z
        - place_position_m has exactly 3 elements (if present)

    Args:
        config: dict — the collision calibration config.

    Returns:
        (is_valid, errors) where is_valid is bool and errors is list[str].
    """
    errors = []

    if 'objects' not in config:
        errors.append("Missing 'objects' key in config")
        return (False, errors)

    objects = config.get('objects', {})

    # Find floor and ceiling objects by type
    floor_objects = [(name, obj) for name, obj in objects.items() if obj.get('type') == 'floor']
    ceiling_objects = [(name, obj) for name, obj in objects.items() if obj.get('type') == 'ceiling']

    if len(floor_objects) == 0:
        errors.append("No floor-type object found (required: exactly 1)")
    elif len(floor_objects) > 1:
        names = [n for n, _ in floor_objects]
        errors.append(f"Multiple floor-type objects found: {names} (required: exactly 1)")

    if len(ceiling_objects) == 0:
        errors.append("No ceiling-type object found (required: exactly 1)")
    elif len(ceiling_objects) > 1:
        names = [n for n, _ in ceiling_objects]
        errors.append(f"Multiple ceiling-type objects found: {names} (required: exactly 1)")

    # Check ceiling_z > floor_z only if we have exactly one of each
    if len(floor_objects) == 1 and len(ceiling_objects) == 1:
        floor_z = floor_objects[0][1].get('z_mm')
        ceiling_z = ceiling_objects[0][1].get('z_mm')
        if floor_z is not None and ceiling_z is not None:
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


# ---------------------------------------------------------------------------
# ROS2 Node — Interactive Collision Calibrator
# ---------------------------------------------------------------------------

import time
import json
import os
import tempfile
import datetime
import threading

try:
    import rclpy
    from rclpy.node import Node as _NodeBase
    from rclpy.executors import MultiThreadedExecutor
except ImportError:
    # Allow geometry-only imports (tests) without ROS2
    rclpy = None  # type: ignore[assignment]
    _NodeBase = object  # type: ignore[assignment,misc]
    MultiThreadedExecutor = None  # type: ignore[assignment,misc]


class CollisionCalibratorNode(_NodeBase):  # type: ignore[misc]
    """Interactive node that guides through calibrating 7 workspace objects."""

    def __init__(self):
        super().__init__('collision_calibrator')
        self._last_tcp = None
        self._tcp_time = 0.0

        # Subscribe to ToolVectorActual for TCP position (mm)
        from dobot_msgs_v4.msg import ToolVectorActual
        self._tcp_sub = self.create_subscription(
            ToolVectorActual,
            '/dobot_msgs_v4/msg/ToolVectorActual',
            self._tcp_callback,
            10
        )

        # Start interactive calibration in daemon thread
        t = threading.Thread(target=self._calibration_loop, daemon=True)
        t.start()

    # ── TCP helpers ───────────────────────────────────────────

    def _tcp_callback(self, msg):
        self._last_tcp = [msg.x, msg.y, msg.z]
        self._tcp_time = time.time()

    def get_tcp(self):
        """Get current TCP position in mm. Raises RuntimeError if no data or stale (>1s)."""
        if self._last_tcp is None:
            raise RuntimeError("Sin lectura TCP — ¿está publicando ToolVectorActual?")
        age = time.time() - self._tcp_time
        if age > 1.0:
            raise RuntimeError(f"Lectura TCP caducada ({age:.1f}s antigua)")
        return list(self._last_tcp)

    # ── UI helpers ────────────────────────────────────────────

    def _wait_enter(self, prompt=""):
        """Show prompt + live TCP, wait for ENTER. Returns tcp_at_press."""
        while True:
            tcp_str = ""
            try:
                tcp = self.get_tcp()
                tcp_str = f"    TCP actual: X={tcp[0]:.1f}  Y={tcp[1]:.1f}  Z={tcp[2]:.1f} mm"
            except RuntimeError:
                tcp_str = "    TCP actual: --- sin lectura ---"
            print(tcp_str)
            if prompt:
                print(f"    {prompt}")
            input("    >> Presiona ENTER para capturar... ")
            try:
                captured = self.get_tcp()
                return captured
            except RuntimeError as e:
                print(f"    ⚠️  Error al capturar: {e}")
                print("    Intenta de nuevo.\n")

    def _prompt_float(self, prompt, min_val=None, max_val=None):
        """Ask user for a float value with optional bounds. Loop until valid."""
        while True:
            try:
                raw = input(f"    {prompt}")
                val = float(raw.strip())
                if min_val is not None and val < min_val:
                    print(f"    ⚠️  Valor debe ser >= {min_val}")
                    continue
                if max_val is not None and val > max_val:
                    print(f"    ⚠️  Valor debe ser <= {max_val}")
                    continue
                return val
            except ValueError:
                print("    ⚠️  Introduce un número válido.")

    def _confirm(self, summary_lines):
        """Show summary_lines, ask confirmation. Return True/False."""
        print()
        for line in summary_lines:
            print(f"    {line}")
        while True:
            choice = input("    ¿Confirmar? (s/n): ").strip().lower()
            if choice in ('s', 'si', 'sí', 'y', 'yes'):
                return True
            if choice in ('n', 'no'):
                return False
            print("    ⚠️  Responde 's' o 'n'.")

    # ── Calibration sequence ──────────────────────────────────

    def _calibrate_suelo(self):
        """Calibrate floor plane."""
        print("\n    === SUELO (Plano del piso) ===")
        print("    Mueve el TCP al nivel del PISO y presiona ENTER...")
        tcp = self._wait_enter()
        print(f"    ✅ Suelo: Z = {tcp[2]:.1f} mm")
        return {'type': 'floor', 'z_mm': tcp[2]}

    def _calibrate_techo(self, suelo_z):
        """Calibrate ceiling plane. Validates techo_z > suelo_z."""
        while True:
            print("\n    === TECHO (Plano del techo) ===")
            print("    Mueve el TCP al nivel del TECHO y presiona ENTER...")
            tcp = self._wait_enter()
            if tcp[2] > suelo_z:
                print(f"    ✅ Techo: Z = {tcp[2]:.1f} mm")
                print(f"    Validación: techo_z ({tcp[2]:.1f}) > suelo_z ({suelo_z:.1f}) ✅")
                return {'type': 'ceiling', 'z_mm': tcp[2]}
            else:
                print(f"    ❌ Error: techo_z ({tcp[2]:.1f}) debe ser > suelo_z ({suelo_z:.1f})")
                print("    Intenta de nuevo.\n")

    def _calibrate_box(self, name, label):
        """Calibrate a box object (mesa, pallet, cinta). Returns dict or re-prompts."""
        while True:
            print(f"\n    === {label} ===")
            print("    ESQUINA 1: Mueve el TCP a la esquina diagonal 1 de la SUPERFICIE y ENTER...")
            p1 = self._wait_enter()
            print(f"    Esquina 1 capturada: [{p1[0]:.1f}, {p1[1]:.1f}, {p1[2]:.1f}] mm")

            print("    ESQUINA 2: Mueve el TCP a la ESQUINA OPUESTA diagonal de la superficie y ENTER...")
            p2 = self._wait_enter()
            print(f"    Esquina 2 capturada: [{p2[0]:.1f}, {p2[1]:.1f}, {p2[2]:.1f}] mm")

            height = self._prompt_float("¿Altura del objeto en mm? (ej: 800): ", min_val=50.0)

            try:
                center_m, dims_m = corners_to_box(p1, p2, height)
                dx = abs(p2[0] - p1[0])
                dy = abs(p2[1] - p1[1])
                confirmed = self._confirm([
                    f"Dimensiones calculadas: Ancho={dx:.0f}mm, Prof={dy:.0f}mm, Alto={height:.0f}mm",
                    f"Centro (m): [{center_m[0]:.3f}, {center_m[1]:.3f}, {center_m[2]:.3f}]",
                    f"Tamaño (m): [{dims_m[0]:.3f}, {dims_m[1]:.3f}, {dims_m[2]:.3f}]",
                ])
                if confirmed:
                    print(f"    ✅ {name} registrado")
                    return {'type': 'box', 'corner1_mm': p1, 'corner2_mm': p2, 'height_mm': height}
                else:
                    print("    🔄 Repitiendo calibración...")
            except ValueError as e:
                print(f"    ❌ Error de validación: {e}")
                print("    Intenta de nuevo.\n")

    def _calibrate_camara(self, techo_z):
        """Calibrate camera support pole."""
        while True:
            print("\n    === SOPORTE DE CÁMARA (Palo vertical) ===")
            print("    Mueve el TCP a la parte INFERIOR del soporte (donde está la cámara) y ENTER...")
            bottom = self._wait_enter()
            print(f"    Punto inferior capturado: [{bottom[0]:.1f}, {bottom[1]:.1f}, {bottom[2]:.1f}] mm")

            width = self._prompt_float("¿Ancho del soporte en mm? (ej: 80): ", min_val=10.0)

            try:
                center_m, dims_m = pole_box(bottom, width, techo_z)
                h_mm = techo_z - bottom[2]
                confirmed = self._confirm([
                    f"Dimensiones: {width:.0f}mm x {width:.0f}mm, Altura={h_mm:.0f}mm (desde Z={bottom[2]:.1f} hasta techo Z={techo_z:.1f})",
                    f"Centro (m): [{center_m[0]:.3f}, {center_m[1]:.3f}, {center_m[2]:.3f}]",
                    f"Tamaño (m): [{dims_m[0]:.3f}, {dims_m[1]:.3f}, {dims_m[2]:.3f}]",
                ])
                if confirmed:
                    print("    ✅ camara_soporte registrado")
                    return {'type': 'pole', 'bottom_mm': bottom, 'width_mm': width}
                else:
                    print("    🔄 Repitiendo calibración...")
            except ValueError as e:
                print(f"    ❌ Error de validación: {e}")
                print("    Intenta de nuevo.\n")

    def _calibrate_pared(self, techo_z, suelo_z):
        """Calibrate wall."""
        while True:
            print("\n    === PARED ===")
            print("    Mueve el TCP al PUNTO 1 de la base de la pared y ENTER...")
            p1 = self._wait_enter()
            print(f"    Punto 1: [{p1[0]:.1f}, {p1[1]:.1f}, {p1[2]:.1f}] mm")

            print("    Mueve el TCP al PUNTO 2 de la base de la pared y ENTER...")
            p2 = self._wait_enter()
            print(f"    Punto 2: [{p2[0]:.1f}, {p2[1]:.1f}, {p2[2]:.1f}] mm")

            try:
                center_m, dims_m, rot = wall_box(p1, p2, techo_z, suelo_z)
                length_mm = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
                confirmed = self._confirm([
                    f"Longitud: {length_mm:.0f}mm | Desde suelo ({suelo_z:.1f}mm) hasta techo ({techo_z:.1f}mm) | Grosor: 100mm",
                    f"Centro (m): [{center_m[0]:.3f}, {center_m[1]:.3f}, {center_m[2]:.3f}]",
                    f"Rotación: {math.degrees(rot):.1f}°",
                ])
                if confirmed:
                    print("    ✅ pared registrada")
                    return {'type': 'wall', 'point1_mm': p1, 'point2_mm': p2, 'thickness_mm': 100}
                else:
                    print("    🔄 Repitiendo calibración...")
            except ValueError as e:
                print(f"    ❌ Error de validación: {e}")
                print("    Intenta de nuevo.\n")

    # ── Main calibration loop (runs in daemon thread) ─────────

    def _calibration_loop(self):
        """Sequential calibration of all 7 workspace objects."""
        try:
            time.sleep(1.5)  # let subscriptions connect
            print()
            print("    ╔═════════════════════════════════════════════════╗")
            print("    ║   CALIBRACIÓN DE COLISIONES — Dobot CR20       ║")
            print("    ╚═════════════════════════════════════════════════╝")
            print()
            print("    Mueve el robot con el teach pendant.")
            print("    Presiona ENTER para capturar la posición actual.")
            print()
            print("    Orden: Suelo → Techo → Mesa → Pallet → Cinta → Cámara → Pared")
            print()

            objects = {}

            # 1. Suelo
            data = self._calibrate_suelo()
            objects['suelo'] = data
            suelo_z = data['z_mm']

            # 2. Techo
            data = self._calibrate_techo(suelo_z)
            objects['techo'] = data
            techo_z = data['z_mm']

            # 3. Mesa de trabajo
            objects['mesa_trabajo'] = self._calibrate_box(
                'mesa_trabajo', 'MESA DE TRABAJO')

            # 4. Pallet de jugos
            objects['pallet_jugos'] = self._calibrate_box(
                'pallet_jugos', 'PALLET DE JUGOS')

            # 5. Cinta transportadora
            cinta = self._calibrate_box(
                'cinta_transportadora', 'CINTA TRANSPORTADORA')
            objects['cinta_transportadora'] = cinta

            # 6. Soporte de cámara
            objects['camara_soporte'] = self._calibrate_camara(techo_z)

            # 7. Pared
            objects['pared'] = self._calibrate_pared(techo_z, suelo_z)

            # ── Derive place position from cinta ──────────────
            place_pos = derive_place_position(
                cinta['corner1_mm'], cinta['corner2_mm'],
                cinta['height_mm'],
                offset_mm=20.0
            )

            # ── Build config ──────────────────────────────────
            config = {
                'status': 'calibrated',
                'timestamp': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'units': 'mm',
                'objects': objects,
                'place_position_m': place_pos,
                'place_position_note': 'Centro de cinta_transportadora + 20mm offset Z. En METROS.',
            }

            # ── Validate before saving ────────────────────────
            is_valid, errs = validate_config(config)
            if not is_valid:
                print("\n    ❌ Validación fallida:")
                for e in errs:
                    print(f"       - {e}")
                print("    NO se guardó ningún archivo.")
                rclpy.shutdown()
                return

            # ── Atomic save ───────────────────────────────────
            config_path = os.path.expanduser('~/dobot_ws/collision_config.json')
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            with tempfile.NamedTemporaryFile('w', dir=os.path.dirname(config_path),
                                             suffix='.tmp', delete=False) as f:
                json.dump(config, f, indent=2)
                tmp_path = f.name
            os.rename(tmp_path, config_path)

            # ── Final summary ─────────────────────────────────
            print()
            print("    ╔═════════════════════════════════════════════════╗")
            print("    ║           CALIBRACIÓN COMPLETADA ✅             ║")
            print("    ╚═════════════════════════════════════════════════╝")
            print()
            print(f"    Archivo: {config_path}")
            print(f"    Timestamp: {config['timestamp']}")
            print()
            print("    Objetos calibrados:")
            for obj_name, obj in objects.items():
                otype = obj['type']
                if otype == 'floor':
                    print(f"      • {obj_name}: Z = {obj['z_mm']:.1f} mm")
                elif otype == 'ceiling':
                    print(f"      • {obj_name}: Z = {obj['z_mm']:.1f} mm")
                elif otype == 'box':
                    print(f"      • {obj_name}: esquinas [{obj['corner1_mm'][0]:.0f},{obj['corner1_mm'][1]:.0f},{obj['corner1_mm'][2]:.0f}] -> [{obj['corner2_mm'][0]:.0f},{obj['corner2_mm'][1]:.0f},{obj['corner2_mm'][2]:.0f}], h={obj['height_mm']:.0f}mm")
                elif otype == 'pole':
                    print(f"      • {obj_name}: base=[{obj['bottom_mm'][0]:.0f},{obj['bottom_mm'][1]:.0f},{obj['bottom_mm'][2]:.0f}], w={obj['width_mm']:.0f}mm")
                elif otype == 'wall':
                    print(f"      • {obj_name}: [{obj['point1_mm'][0]:.0f},{obj['point1_mm'][1]:.0f}] -> [{obj['point2_mm'][0]:.0f},{obj['point2_mm'][1]:.0f}], grosor={obj['thickness_mm']}mm")
            print()
            print(f"    Place position (m): [{place_pos[0]:.4f}, {place_pos[1]:.4f}, {place_pos[2]:.4f}]")
            print()

            # Signal executor to stop (main() handles shutdown)
            raise SystemExit(0)

        except KeyboardInterrupt:
            print("\n\n    ⚠️  Calibración interrumpida — NO se guardó ningún archivo.")
            raise SystemExit(0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    import rclpy as _rclpy
    from rclpy.executors import MultiThreadedExecutor as _MTE
    _rclpy.init(args=args)
    node = None
    try:
        node = CollisionCalibratorNode()
        executor = _MTE(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            _rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

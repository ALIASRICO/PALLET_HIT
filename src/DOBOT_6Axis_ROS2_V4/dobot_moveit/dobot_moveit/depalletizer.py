#!/usr/bin/env python3
"""
Despaletizador — modo prueba
- Orientación TCP completamente fija (ventosa siempre abajo, yaw fijo)
- Sin gripper — solo movimientos de brazo
- Bloquea hasta que MoveIt termine cada movimiento
"""

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
import json
import os
import time
import threading

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from dobot_msgs_v4.msg import ToolVectorActual


def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    r  = math.radians(roll_deg)
    p  = math.radians(pitch_deg)
    y  = math.radians(yaw_deg)
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return {
        'x': sr*cp*cy - cr*sp*sy,
        'y': cr*sp*cy + sr*cp*sy,
        'z': cr*cp*sy - sr*sp*cy,
        'w': cr*cp*cy + sr*sp*sy,
    }


# ── Orientación fija del TCP ──────────────────────────────────────────────────
TCP_FIXED_QUAT = rpy_to_quaternion(177.0, -0.38, 0.53)

ORI_TOL_ROLL  = 0.05
ORI_TOL_PITCH = 0.05
ORI_TOL_YAW   = 0.05

# ── Home Position (joint-space, degrees → radians) ────────────────────────────
HOME_JOINT_DEG = [0, -30, 90, 0, -60, 0]   # arm folded forward/down, below ceiling
HOME_JOINT_RAD = [math.radians(d) for d in HOME_JOINT_DEG]

JUICE_TYPE_MANGO   = 0.0
JUICE_TYPE_MORA    = 1.0
JUICE_TYPE_UNKNOWN = -1.0


class DepalletizerNode(Node):
    def __init__(self):
        super().__init__('depalletizer')
        self.cb_group = ReentrantCallbackGroup()

        self.get_logger().info('=' * 60)
        self.get_logger().info('   🧃 DESPALETIZADOR — MODO PRUEBA 🧃')
        self.get_logger().info('   Orientación TCP fija | Sin gripper')
        self.get_logger().info('=' * 60)

        # ── Config ───────────────────────────────────────────
        self.calibration_file  = os.path.expanduser('~/dobot_ws/calibration_config.json')
        self.z_approach        = 0.15
        self.z_lift            = 0.20
        self.z_pick_extra      = 0.00
        self.speed_scaling     = 0.2
        self.place_position    = [0.5, -0.4, 0.15]

        _collision_cfg = os.path.expanduser('~/dobot_ws/collision_config.json')
        if os.path.exists(_collision_cfg):
            try:
                with open(_collision_cfg, 'r') as _f:
                    _cc = json.load(_f)
                _pp = _cc.get('place_position_m')
                if _pp and len(_pp) == 3:
                    self.place_position = [float(_pp[0]), float(_pp[1]), float(_pp[2])]
                    self.get_logger().info(
                        f'✅ place_position cargado de collision_config: '
                        f'({self.place_position[0]:.3f}, {self.place_position[1]:.3f}, {self.place_position[2]:.3f})m')
                else:
                    self.get_logger().warn('⚠️ collision_config.json sin place_position_m válido — usando default')
            except Exception as _e:
                self.get_logger().warn(f'⚠️ Error leyendo collision_config.json: {_e} — usando place_position default')
        else:
            self.get_logger().warn(
                f'⚠️ No existe collision_config.json — place_position default: {self.place_position}')

        self.top_offset_mm     = 20.0
        self.h_max_mm          = 400.0
        self.xy_verify_tol_mm  = 15.0

        # ── Estado ───────────────────────────────────────────
        self.detections        = []
        self.detection_lock    = threading.Lock()
        self.calib             = None
        self.M_xy              = None
        self.z_plane_cam       = None
        self.z_plane_robot     = None
        self.z_work_fallback_m = None
        self.is_busy           = False
        self.running           = True
        self.picked_count      = 0
        self.juice_counts      = {JUICE_TYPE_MANGO: 0, JUICE_TYPE_MORA: 0, JUICE_TYPE_UNKNOWN: 0}
        self.current_joints    = None
        self._last_tcp         = None

        self.planning_group    = 'cr20_group'
        self.end_effector_link = 'Link6'

        self.load_calibration()

        # ── Suscriptores ─────────────────────────────────────
        self.create_subscription(
            Float32MultiArray, '/detections/jugos',
            self.detections_callback, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            JointState, '/joint_states',
            self.joint_states_callback, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            ToolVectorActual, '/dobot_msgs_v4/msg/ToolVectorActual',
            self._tcp_callback, 10,
            callback_group=self.cb_group)

        # ── MoveGroup ────────────────────────────────────────
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.cb_group)

        self.get_logger().info('Esperando MoveGroup...')
        if self.move_group_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().info('✅ MoveGroup conectado')
        else:
            self.get_logger().error('❌ MoveGroup no disponible — verifica que está corriendo')

        # ── Pre-flight: esperar joint states válidos ──────────────
        if not self._wait_for_valid_joint_states(timeout_sec=15.0):
            self.get_logger().error('⚠️ Timeout esperando joint states válidos — continuar con precaución')

        # ── UI thread ────────────────────────────────────────
        self.ui_thread = threading.Thread(target=self.user_interface)
        self.ui_thread.daemon = True
        self.ui_thread.start()

    # ─────────────────────────────────────────────────────────
    # Calibración
    # ─────────────────────────────────────────────────────────
    def load_calibration(self):
        if not os.path.exists(self.calibration_file):
            self.get_logger().error(f'❌ No existe calibración: {self.calibration_file}')
            return
        try:
            with open(self.calibration_file, 'r') as f:
                self.calib = json.load(f)
            self.M_xy          = np.array(self.calib['affine_matrix_2x3'])
            self.z_plane_cam   = self.calib.get('z_plane_cam', None)
            self.z_plane_robot = self.calib.get('z_plane_robot', None)
            zfb = self.calib.get('z_work_mm_fallback', None)
            self.z_work_fallback_m = float(zfb) / 1000.0 if zfb is not None else None
            self.get_logger().info('✅ Calibración cargada')
            self.get_logger().info(f'   z_plane_cam:   {"OK" if self.z_plane_cam   else "NO"}')
            self.get_logger().info(f'   z_plane_robot: {"OK" if self.z_plane_robot else "NO"}')
        except Exception as e:
            self.get_logger().error(f'❌ Error cargando calibración: {e}')

    def eval_plane(self, plane, x_mm, y_mm):
        return plane['a'] * x_mm + plane['b'] * y_mm + plane['c']

    def camxy_to_robotxy_mm(self, cam_x_mm, cam_y_mm):
        M  = self.M_xy
        rx = M[0,0]*cam_x_mm + M[0,1]*cam_y_mm + M[0,2]
        ry = M[1,0]*cam_x_mm + M[1,1]*cam_y_mm + M[1,2]
        return rx, ry

    def compute_grasp_z_robot_mm(self, cam_x_mm, cam_y_mm, cam_z_mm,
                                  robot_x_mm, robot_y_mm):
        if self.z_plane_cam is None or self.z_plane_robot is None:
            if self.z_work_fallback_m is None:
                return None, None, None, None
            fb = self.z_work_fallback_m * 1000.0
            return fb, 0.0, fb, None
        z_surf_cam   = self.eval_plane(self.z_plane_cam, cam_x_mm, cam_y_mm)
        h            = max(0.0, min(z_surf_cam - cam_z_mm, self.h_max_mm))
        z_surf_robot = self.eval_plane(self.z_plane_robot, robot_x_mm, robot_y_mm)
        z_grasp      = z_surf_robot + max(0.0, h - self.top_offset_mm)
        return z_grasp, h, z_surf_robot, z_surf_cam

    # ─────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────
    def detections_callback(self, msg):
        data = msg.data
        with self.detection_lock:
            self.detections.clear()
            for i in range(0, len(data), 8):
                if i + 7 >= len(data):
                    continue
                det_id   = int(data[i])
                cam_x_mm = float(data[i+1])
                cam_y_mm = float(data[i+2])
                cam_z_mm = float(data[i+3])
                conf     = float(data[i+4])
                det = {'id': det_id, 'confidence': conf,
                       'cam_x_mm': cam_x_mm, 'cam_y_mm': cam_y_mm, 'cam_z_mm': cam_z_mm}
                det['juice_type'] = float(data[i+7])
                if self.M_xy is not None:
                    rx_mm, ry_mm = self.camxy_to_robotxy_mm(cam_x_mm, cam_y_mm)
                    z_grasp_mm, h_mm, z_surf_robot_mm, z_surf_cam_mm = \
                        self.compute_grasp_z_robot_mm(cam_x_mm, cam_y_mm, cam_z_mm, rx_mm, ry_mm)
                    det['robot_x']       = rx_mm / 1000.0
                    det['robot_y']       = ry_mm / 1000.0
                    det['robot_z_grasp'] = (z_grasp_mm / 1000.0) if z_grasp_mm is not None else None
                    det['h_obj_mm']      = h_mm
                self.detections.append(det)

    def joint_states_callback(self, msg):
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position[:6])

    def _tcp_callback(self, msg):
        self._last_tcp = {'x': msg.x, 'y': msg.y, 'z': msg.z}

    # ─────────────────────────────────────────────────────────
    # Pre-flight: validación de joint states
    # ─────────────────────────────────────────────────────────
    def _wait_for_valid_joint_states(self, timeout_sec=15.0):
        """Espera hasta que /joint_states reporte posiciones no-cero."""
        self._js_valid = False
        _valid_pos = [None]

        def _js_check_cb(msg):
            if len(msg.position) >= 6:
                if not all(abs(p) < 1e-6 for p in msg.position):
                    self._js_valid = True
                    _valid_pos[0] = list(msg.position[:6])

        sub = self.create_subscription(
            JointState, '/joint_states',
            _js_check_cb, 10,
            callback_group=self.cb_group)

        t0 = time.time()
        last_log_time = 0.0
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._js_valid:
                pos_str = ', '.join(f'{p:.4f}' for p in _valid_pos[0])
                self.get_logger().info(f'✅ Joint states válidos recibidos: [{pos_str}]')
                self.destroy_subscription(sub)
                return True
            now = time.time()
            if now - last_log_time >= 2.0:
                elapsed = now - t0
                self.get_logger().warn(
                    f'⏳ Esperando joint states válidos... ({elapsed:.1f}s/{timeout_sec:.1f}s)')
                last_log_time = now

        self.destroy_subscription(sub)
        return False

    # ─────────────────────────────────────────────────────────
    # Helper: esperar future sin spin propio (el executor ya spinea)
    # ─────────────────────────────────────────────────────────
    def _wait_future(self, future, timeout_sec=30.0):
        """Polling del future — compatible con MultiThreadedExecutor en marcha."""
        deadline = time.time() + timeout_sec
        while not future.done():
            if time.time() > deadline:
                return False
            time.sleep(0.05)
        return True

    # ─────────────────────────────────────────────────────────
    # MoveIt — orientación 100% fija
    # ─────────────────────────────────────────────────────────
    def move_to_pose(self, x, y, z, label=''):
        tag = f'[{label}] ' if label else ''
        self.get_logger().info(f'📍 {tag}Moviendo a: ({x:.3f}, {y:.3f}, {z:.3f})m')

        goal = MoveGroup.Goal()
        goal.request.group_name                      = self.planning_group
        goal.request.num_planning_attempts           = 20
        goal.request.allowed_planning_time           = 10.0
        goal.request.max_velocity_scaling_factor     = self.speed_scaling
        goal.request.max_acceleration_scaling_factor = self.speed_scaling

        constraints = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name       = self.end_effector_link
        sphere             = SolidPrimitive()
        sphere.type        = SolidPrimitive.SPHERE
        sphere.dimensions  = [0.01]
        pc.constraint_region.primitives.append(sphere)
        rp               = Pose()
        rp.position.x    = x
        rp.position.y    = y
        rp.position.z    = z
        rp.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(rp)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id           = 'base_link'
        oc.link_name                 = self.end_effector_link
        oc.orientation.x             = TCP_FIXED_QUAT['x']
        oc.orientation.y             = TCP_FIXED_QUAT['y']
        oc.orientation.z             = TCP_FIXED_QUAT['z']
        oc.orientation.w             = TCP_FIXED_QUAT['w']
        oc.absolute_x_axis_tolerance = ORI_TOL_ROLL
        oc.absolute_y_axis_tolerance = ORI_TOL_PITCH
        oc.absolute_z_axis_tolerance = ORI_TOL_YAW
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(f'   ⏳ Planeando...')
        future = self.move_group_client.send_goal_async(goal)
        if not self._wait_future(future, timeout_sec=15.0):
            self.get_logger().error('❌ Timeout en aceptación del goal')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rechazado por MoveGroup')
            return False

        self.get_logger().info('   ⏳ Ejecutando trayectoria...')
        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=120.0):
            self.get_logger().error('❌ Timeout en ejecución')
            return False

        result = result_future.result().result
        code   = result.error_code.val

        if code == MoveItErrorCodes.SUCCESS:
            tcp = self._last_tcp
            if tcp:
                self.get_logger().info(
                    f'   ✅ Llegó → TCP real: ({tcp["x"]:.1f}, {tcp["y"]:.1f}, {tcp["z"]:.1f})mm')
            else:
                self.get_logger().info('   ✅ Movimiento completado')
            return True

        error_names = {
            -1:  'FAILURE',
            -2:  'PLANNING_FAILED (IK no encontrada)',
            -3:  'INVALID_MOTION_PLAN',
            -4:  'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
            -5:  'CONTROL_FAILED',
            -6:  'UNABLE_TO_AQUIRE_SENSOR_DATA',
            -7:  'TIMED_OUT',
            -8:  'PREEMPTED',
            -9:  'START_STATE_IN_COLLISION',
            -10: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
            -11: 'GOAL_IN_COLLISION',
            -12: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
            -13: 'GOAL_CONSTRAINTS_VIOLATED',
            -14: 'INVALID_GROUP_NAME',
            -15: 'INVALID_GOAL_CONSTRAINTS',
            -16: 'INVALID_ROBOT_STATE',
            -17: 'INVALID_LINK_NAME',
            -18: 'INVALID_OBJECT_NAME',
        }
        desc = error_names.get(code, f'código {code}')
        self.get_logger().error(f'❌ MoveIt falló: {desc}')
        return False

    # ─────────────────────────────────────────────────────────
    # Home position — joint-space goal
    # ─────────────────────────────────────────────────────────
    def go_home(self):
        self.get_logger().info('🏠 Moviendo a posición Home...')
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        goal = MoveGroup.Goal()
        goal.request.group_name                      = self.planning_group
        goal.request.num_planning_attempts           = 20
        goal.request.allowed_planning_time           = 10.0
        goal.request.max_velocity_scaling_factor     = self.speed_scaling
        goal.request.max_acceleration_scaling_factor = self.speed_scaling

        constraints = Constraints()
        for name, position in zip(joint_names, HOME_JOINT_RAD):
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info('   ⏳ Planeando hacia Home...')
        future = self.move_group_client.send_goal_async(goal)
        if not self._wait_future(future, timeout_sec=15.0):
            self.get_logger().error('❌ Timeout en aceptación del goal (Home)')
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal Home rechazado por MoveGroup')
            return False

        self.get_logger().info('   ⏳ Ejecutando trayectoria Home...')
        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=120.0):
            self.get_logger().error('❌ Timeout en ejecución Home')
            return False

        result = result_future.result().result
        code   = result.error_code.val

        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('   ✅ Robot en posición Home')
            return True

        error_names = {
            -1: 'FAILURE', -2: 'PLANNING_FAILED', -9: 'START_STATE_IN_COLLISION',
            -10: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
        }
        desc = error_names.get(code, f'código {code}')
        self.get_logger().error(f'❌ MoveIt falló (Home): {desc}')
        return False

    # ─────────────────────────────────────────────────────────
    # Verificación XY
    # ─────────────────────────────────────────────────────────
    def verify_xy(self, target_x_m, target_y_m):
        if self._last_tcp is None:
            self.get_logger().warn('⚠️ Sin lectura TCP, omitiendo verificación')
            return True
        err_x = abs(self._last_tcp['x'] - target_x_m * 1000.0)
        err_y = abs(self._last_tcp['y'] - target_y_m * 1000.0)
        self.get_logger().info(
            f'   📐 Error XY → dx={err_x:.1f}mm  dy={err_y:.1f}mm  '
            f'(tol={self.xy_verify_tol_mm}mm)')
        if err_x > self.xy_verify_tol_mm or err_y > self.xy_verify_tol_mm:
            self.get_logger().error('❌ Error XY fuera de tolerancia — abortando')
            return False
        self.get_logger().info('   ✅ XY dentro de tolerancia')
        return True

    # ─────────────────────────────────────────────────────────
    # Ciclo pick & place
    # ─────────────────────────────────────────────────────────
    def pick_and_place_cycle(self):
        if not self._wait_for_valid_joint_states(timeout_sec=5.0):
            self.get_logger().error('❌ Joint states aún en ceros — ¿está dobot_bringup corriendo?')
            return

        if self.is_busy or self.M_xy is None:
            return False

        with self.detection_lock:
            if not self.detections:
                self.get_logger().warn('⚠️ Sin detecciones')
                return False
            Z_LAYER_TOL_MM = 15.0
            sorted_dets = sorted(self.detections, key=lambda d: d.get('cam_z_mm', 9999.0))
            best = sorted_dets[0]
            best_z = best.get('cam_z_mm', 0.0)
            same_layer = [d for d in sorted_dets
                          if abs(d.get('cam_z_mm', 0.0) - best_z) <= Z_LAYER_TOL_MM]
            if len(same_layer) > 1:
                same_layer.sort(key=lambda d: math.sqrt(
                    (d.get('robot_x', 0.0) * 1000) ** 2 +
                    (d.get('robot_y', 0.0) * 1000) ** 2))
            target = same_layer[0].copy()

        if target.get('robot_z_grasp') is None:
            self.get_logger().error('❌ Z de agarre no calculada')
            return False

        self.is_busy = True
        try:
            x       = target['robot_x']
            y       = target['robot_y']
            z_grasp = target['robot_z_grasp'] + self.z_pick_extra

            self.get_logger().info('─' * 50)
            _juice_name = {JUICE_TYPE_MANGO: 'MANGO', JUICE_TYPE_MORA: 'MORA',
                           JUICE_TYPE_UNKNOWN: 'DESCONOCIDO'}.get(
                               target.get('juice_type', JUICE_TYPE_UNKNOWN), '???')
            self.get_logger().info(
                f"🎯 Target #{target['id']} [{_juice_name}] | "
                f"XY=({x*1000:.0f},{y*1000:.0f})mm | "
                f"h={target.get('h_obj_mm',0):.1f}mm | "
                f"z_grasp={z_grasp:.3f}m")
            self.get_logger().info('─' * 50)

            # PICK
            self.get_logger().info('   [Sin gripper] Iniciando movimiento...')

            if not self.move_to_pose(x, y, z_grasp + self.z_approach, label='APPROACH'):
                return False
            if not self.verify_xy(x, y):
                return False
            if not self.move_to_pose(x, y, z_grasp, label='DESCENSO'):
                return False

            self.get_logger().info('   [Sin gripper] En posición de agarre')

            if not self.move_to_pose(x, y, z_grasp + self.z_lift, label='LIFT'):
                return False

            # PLACE
            px, py, pz = self.place_position

            if not self.move_to_pose(px, py, pz + 0.1, label='PLACE APPROACH'):
                return False
            if not self.move_to_pose(px, py, pz, label='PLACE'):
                return False

            self.get_logger().info('   [Sin gripper] En posición de depósito')

            if not self.move_to_pose(px, py, pz + 0.1, label='PLACE LIFT'):
                return False

            self.picked_count += 1
            _jt = target.get('juice_type', JUICE_TYPE_UNKNOWN)
            self.juice_counts[_jt] = self.juice_counts.get(_jt, 0) + 1
            _jname = {JUICE_TYPE_MANGO: 'MANGO', JUICE_TYPE_MORA: 'MORA',
                      JUICE_TYPE_UNKNOWN: 'DESCONOCIDO'}.get(_jt, '???')
            self.get_logger().info(
                f'🎉 Ciclo completado. Total: {self.picked_count} | 🧃 Tipo: {_jname}')

        except Exception as e:
            self.get_logger().error(f'❌ Excepción: {e}')
            return False
        finally:
            self.is_busy = False

        return True

    # ─────────────────────────────────────────────────────────
    # UI
    # ─────────────────────────────────────────────────────────
    def user_interface(self):
        time.sleep(2.0)
        self.print_menu()
        while self.running:
            try:
                choice = input('\n    Opción: ').strip().lower()
                if choice == '1':
                    self.show_detections()
                elif choice == '2':
                    self.pick_and_place_cycle()
                elif choice == 'h':
                    self.go_home()
                elif choice == 'q':
                    self.running = False
                    rclpy.shutdown()
                else:
                    self.print_menu()
            except Exception:
                break

    def print_menu(self):
        print('\n    ╔═══════════════════════════════════╗')
        print('    ║   🧃 DESPALETIZADOR — PRUEBA 🧃   ║')
        print('    ╠═══════════════════════════════════╣')
        print('    ║  1 = Ver detecciones              ║')
        print('    ║  2 = Pick & Place                 ║')
        print('    ║  H = Ir a Home                    ║')
        print('    ║  Q = Salir                        ║')
        print('    ╚═══════════════════════════════════╝')
        ok = self.M_xy is not None
        tcp = self._last_tcp
        tcp_str = f'TCP:({tcp["x"]:.0f},{tcp["y"]:.0f},{tcp["z"]:.0f})' if tcp else 'TCP:---'
        print(f"    Calib:{'✅' if ok else '❌'} | {tcp_str} | Picks:{self.picked_count}")

    def show_detections(self):
        with self.detection_lock:
            print(f'\n    Detecciones: {len(self.detections)}')
            for d in self.detections:
                if 'robot_x' in d:
                    _jn = {JUICE_TYPE_MANGO: 'MANGO', JUICE_TYPE_MORA: 'MORA',
                           JUICE_TYPE_UNKNOWN: '???'}.get(d.get('juice_type', JUICE_TYPE_UNKNOWN), '???')
                    print(
                        f"    🧃 #{d['id']} [{_jn}] conf={d['confidence']:.2f} | "
                        f"R=({d['robot_x']*1000:.0f},{d['robot_y']*1000:.0f})mm | "
                        f"h={d.get('h_obj_mm',0):.1f}mm | "
                        f"z_grasp={d.get('robot_z_grasp',0):.3f}m")

    def destroy_node(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('  📊 INFORME DE JUGOS RECOGIDOS')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'  🥭 MANGO:      {self.juice_counts.get(JUICE_TYPE_MANGO, 0)}')
        self.get_logger().info(f'  🫐 MORA:        {self.juice_counts.get(JUICE_TYPE_MORA, 0)}')
        self.get_logger().info(f'  ❓ DESCONOCIDO: {self.juice_counts.get(JUICE_TYPE_UNKNOWN, 0)}')
        self.get_logger().info(f'  📦 TOTAL:       {self.picked_count}')
        self.get_logger().info('=' * 50)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node     = DepalletizerNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

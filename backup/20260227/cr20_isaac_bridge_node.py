#!/usr/bin/env python3
"""
================================================================
CR20 Isaac Sim Bridge Node
================================================================

Nodo ROS2 que actúa como bridge entre ROS2 y NVIDIA Isaac Sim para 
el robot Dobot CR20.

Funcionalidades:
- Recibe comandos de trayectoria de MoveIt
- Publica estados articulares para MoveIt
- Convierte comandos ROS2 a formato Isaac Sim
- Lee estados de Isaac Sim y los publica en ROS2

Uso:
    ros2 run cr20_moveit cr20_isaac_bridge_node.py

    # Con parámetros
    ros2 run cr20_moveit cr20_isaac_bridge_node.py \
        --ros-args \
        -p robot_name:=cr20_robot \
        -p isaac_address:=127.0.0.1:5555

================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

# Mensajes ROS2
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from std_msgs.msg import Float64, Header, Bool
from rosgraph_msgs.msg import Clock

# Bibliotecas estándar
import numpy as np
import threading
import time
import json
import socket
from typing import List, Dict, Optional, Any


class CR20IsaacBridge(Node):
    """
    Nodo bridge entre ROS2 y Isaac Sim para el robot CR20.
    
    Este nodo:
    1. Suscribe a comandos de trayectoria (/joint_trajectory)
    2. Publica estados (/joint_states)
    3. Conecta con Isaac Sim vía socket TCP o ROS bridge
    4. Convierte entre formatos ROS2 e Isaac Sim
    """
    
    # Nombres de joints del CR20
    JOINT_NAMES = [
        'joint1',
        'joint2', 
        'joint3',
        'joint4',
        'joint5',
        'joint6'
    ]
    
    # Límites de movimiento (desde URDF)
    JOINT_LIMITS = {
        'joint1': {'min': -6.28, 'max': 6.28, 'max_velocity': 3.14, 'max_effort': 150.0},
        'joint2': {'min': -6.28, 'max': 6.28, 'max_velocity': 3.14, 'max_effort': 150.0},
        'joint3': {'min': -2.9, 'max': 2.9, 'max_velocity': 3.14, 'max_effort': 150.0},
        'joint4': {'min': -6.28, 'max': 6.28, 'max_velocity': 3.14, 'max_effort': 50.0},
        'joint5': {'min': -6.28, 'max': 6.28, 'max_velocity': 3.14, 'max_effort': 50.0},
        'joint6': {'min': -6.28, 'max': 6.28, 'max_velocity': 3.14, 'max_effort': 50.0},
    }
    
    def __init__(self):
        """Inicializa el nodo bridge."""
        
        # ============================================
        # 1. INICIALIZACIÓN BÁSICA
        # ============================================
        super().__init__('cr20_isaac_bridge')
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("CR20 Isaac Sim Bridge Node")
        self.get_logger().info("=" * 50)
        
        # ============================================
        # 2. PARÁMETROS
        # ============================================
        self.declare_parameter('robot_name', 'cr20_robot')
        self.declare_parameter('isaac_address', '127.0.0.1:5555')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('command_timeout', 1.0)  # segundos
        self.declare_parameter('enable_isaac', True)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.isaac_address = self.get_parameter('isaac_address').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.enable_isaac = self.get_parameter('enable_isaac').value
        
        self.get_logger().info(f"Robot: {self.robot_name}")
        self.get_logger().info(f"Isaac address: {self.isaac_address}")
        self.get_logger().info(f"Publish rate: {self.publish_rate} Hz")
        
        # ============================================
        # 3. ESTADO INTERNO
        # ============================================
        # Estado actual de los joints
        self.current_positions = np.zeros(6)
        self.current_velocities = np.zeros(6)
        self.current_efforts = np.zeros(6)
        
        # Comando objetivo (desde MoveIt)
        self.target_positions = np.zeros(6)
        self.target_velocities = np.zeros(6)
        self.is_executing = False
        
        # Thread lock para seguridad
        self._lock = threading.Lock()
        
        # Bandera de conexión
        self.isaac_connected = False
        
        # ============================================
        # 4. PUBLISHERS
        # ============================================
        # QoS para publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher de estados articulares
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            qos_profile
        )
        self.get_logger().info("Publisher /joint_states created")
        
        # Publisher de feedback de ejecución
        self.feedback_publisher = self.create_publisher(
            Bool,
            '/execution_feedback',
            qos_profile
        )
        self.get_logger().info("Publisher /execution_feedback created")
        
        # Publisher de clock (si usa tiempo de simulación)
        if self.use_sim_time:
            self.clock_publisher = self.create_publisher(
                Clock,
                '/clock',
                qos_profile
            )
            self.get_logger().info("Publisher /clock created")
        
        # ============================================
        # 5. SUBSCRIBERS
        # ============================================
        # Suscriptor de comandos de trayectoria
        self.trajectory_subscriber = self.create_subscription(
            JointTrajectory,
            f'/{self.robot_name}/joint_trajectory/command',
            self.trajectory_callback,
            qos_profile
        )
        self.get_logger().info(f"Subscribed to /{self.robot_name}/joint_trajectory/command")
        
        # Suscriptor directo de comandos (alternativo)
        self.direct_cmd_subscriber = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory/command',
            self.trajectory_callback,
            qos_profile
        )
        self.get_logger().info("Subscribed to /joint_trajectory/command")
        
        # Suscriptor de comandos individuales
        for i, joint_name in enumerate(self.JOINT_NAMES):
            self.create_subscription(
                Float64,
                f'/{joint_name}/command',
                lambda msg, idx=i: self.single_joint_callback(msg, idx),
                qos_profile
            )
        
        # ============================================
        # 6. TIMERS
        # ============================================
        # Timer para publicar estados
        timer_period = 1.0 / self.publish_rate
        self.state_timer = self.create_timer(
            timer_period,
            self.publish_state
        )
        self.get_logger().info(f"State timer created: {timer_period}s period")
        
        # Timer para verificar conexión
        self.connection_timer = self.create_timer(
            5.0,
            self.check_connection
        )
        
        # ============================================
        # 7. CONEXIÓN ISAAC SIM
        # ============================================
        if self.enable_isaac:
            self._connect_isaac()
        else:
            self.get_logger().warn("Isaac Sim integration disabled")
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("Node initialized successfully")
        self.get_logger().info("=" * 50)
    
    def _connect_isaac(self):
        """
        Inicia conexión con Isaac Sim.
        
        Métodos de conexión:
        1. Socket TCP (directo)
        2. ROS bridge (a través de ros_ign_bridge)
        3. Isaac SDK (Python bindings)
        """
        try:
            # Intentar conexión TCP con Isaac
            addr, port = self.isaac_address.split(':')
            port = int(port)
            
            self.get_logger().info(f"Connecting to Isaac Sim at {addr}:{port}")
            
            # Crear socket TCP
            self.isaac_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.isaac_socket.settimeout(5.0)
            
            try:
                self.isaac_socket.connect((addr, port))
                self.isaac_connected = True
                self.get_logger().info("Connected to Isaac Sim via TCP")
            except socket.timeout:
                self.get_logger().warn("Connection to Isaac Sim timed out")
                self.get_logger().info("Will use simulated mode")
                self.isaac_connected = False
            except Exception as e:
                self.get_logger().warn(f"Could not connect to Isaac Sim: {e}")
                self.get_logger().info("Will use simulated mode")
                self.isaac_connected = False
                
        except Exception as e:
            self.get_logger().error(f"Error connecting to Isaac Sim: {e}")
            self.isaac_connected = False
    
    def trajectory_callback(self, msg: JointTrajectory):
        """
        Callback para recibir comandos de trayectoria de MoveIt.
        
        Args:
            msg: Mensaje JointTrajectory de MoveIt
        """
        self.get_logger().info(
            f"Received trajectory command: {len(msg.points)} points"
        )
        
        if not msg.points:
            self.get_logger().warn("Empty trajectory received")
            return
        
        # Verificar que los nombres de joints son correctos
        if msg.joint_names != self.JOINT_NAMES:
            self.get_logger().warn(
                f"Joint names mismatch. Expected: {self.JOINT_NAMES}, "
                f"Got: {msg.joint_names}"
            )
        
        with self._lock:
            # Tomar el primer punto como objetivo
            point = msg.points[0]
            
            # Verificar longitud
            if len(point.positions) != len(self.JOINT_NAMES):
                self.get_logger().error(
                    f"Invalid trajectory point: expected {len(self.JOINT_NAMES)} "
                    f"positions, got {len(point.positions)}"
                )
                return
            
            # Aplicar límites
            self.target_positions = np.array(point.positions)
            self._apply_joint_limits()
            
            # Velocidades (si están disponibles)
            if point.velocities:
                self.target_velocities = np.array(point.velocities)
            
            self.is_executing = True
        
        # Enviar a Isaac Sim
        if self.isaac_connected:
            self._send_to_isaac(self.target_positions)
        
        self.get_logger().info(
            f"Target positions: {self.target_positions}"
        )
    
    def single_joint_callback(self, msg: Float64, joint_index: int):
        """
        Callback para comandos de joints individuales.
        
        Args:
            msg: Comando de posición
            joint_index: Índice del joint (0-5)
        """
        joint_name = self.JOINT_NAMES[joint_index]
        
        with self._lock:
            # Verificar límites
            limits = self.JOINT_LIMITS[joint_name]
            position = np.clip(msg.data, limits['min'], limits['max'])
            
            self.target_positions[joint_index] = position
            self._apply_joint_limits()
        
        self.get_logger().debug(
            f"Joint {joint_name} command: {position:.4f}"
        )
        
        # Enviar a Isaac Sim
        if self.isaac_connected:
            self._send_single_to_isaac(joint_index, position)
    
    def _apply_joint_limits(self):
        """Aplica límites de movimiento a las posiciones objetivo."""
        for i, joint_name in enumerate(self.JOINT_NAMES):
            limits = self.JOINT_LIMITS[joint_name]
            self.target_positions[i] = np.clip(
                self.target_positions[i],
                limits['min'],
                limits['max']
            )
    
    def _send_to_isaac(self, positions: np.ndarray):
        """
        Envía comandos de posiciones a Isaac Sim.
        
        Args:
            positions: Array de 6 posiciones
        """
        if not self.isaac_connected:
            return
        
        try:
            # Formato de mensaje para Isaac
            # Esto depende de cómo esté configurado Isaac
            message = {
                'type': 'joint_command',
                'positions': positions.tolist(),
                'joint_names': self.JOINT_NAMES
            }
            
            # Enviar como JSON
            data = json.dumps(message) + '\n'
            self.isaac_socket.send(data.encode('utf-8'))
            
        except Exception as e:
            self.get_logger().error(f"Error sending to Isaac: {e}")
            self.isaac_connected = False
    
    def _send_single_to_isaac(self, joint_index: int, position: float):
        """
        Envía comando de joint individual a Isaac Sim.
        
        Args:
            joint_index: Índice del joint
            position: Posición objetivo
        """
        if not self.isaac_connected:
            return
        
        try:
            message = {
                'type': 'single_joint_command',
                'joint_index': joint_index,
                'joint_name': self.JOINT_NAMES[joint_index],
                'position': float(position)
            }
            
            data = json.dumps(message) + '\n'
            self.isaac_socket.send(data.encode('utf-8'))
            
        except Exception as e:
            self.get_logger().error(f"Error sending to Isaac: {e}")
            self.isaac_connected = False
    
    def _read_from_isaac(self) -> Optional[Dict]:
        """
        Lee estados de Isaac Sim.
        
        Returns:
            Dict con estados o None si hay error
        """
        if not self.isaac_connected:
            return None
        
        try:
            # Establecer timeout corto para no bloquear
            self.isaac_socket.settimeout(0.1)
            
            try:
                data = self.isaac_socket.recv(4096)
                if data:
                    return json.loads(data.decode('utf-8'))
            except socket.timeout:
                pass
                
        except Exception as e:
            self.get_logger().debug(f"Error reading from Isaac: {e}")
        
        return None
    
    def publish_state(self):
        """
        Timer callback: publica el estado actual de los joints.
        
        Publica en /joint_states para que MoveIt y otros nodos lo consuman.
        """
        # Leer de Isaac Sim si está conectado
        isaac_state = self._read_from_isaac()
        
        if isaac_state and 'positions' in isaac_state:
            with self._lock:
                self.current_positions = np.array(isaac_state['positions'])
                if 'velocities' in isaac_state:
                    self.current_velocities = np.array(isaac_state['velocities'])
                if 'efforts' in isaac_state:
                    self.current_efforts = np.array(isaac_state['efforts'])
        else:
            # Modo simulado: interpolar hacia objetivo
            with self._lock:
                if self.is_executing:
                    # Interpolación simple (mover 5% hacia objetivo por tick)
                    diff = self.target_positions - self.current_positions
                    self.current_positions += diff * 0.05
                    
                    # Verificar si llegamos
                    if np.max(np.abs(diff)) < 0.001:
                        self.is_executing = False
                        
                        # Publicar feedback de completado
                        feedback = Bool()
                        feedback.data = True
                        self.feedback_publisher.publish(feedback)
        
        # Crear mensaje JointState
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = self.robot_name
        
        joint_state.name = self.JOINT_NAMES
        joint_state.position = self.current_positions.tolist()
        joint_state.velocity = self.current_velocities.tolist()
        joint_state.effort = self.current_efforts.tolist()
        
        # Publicar
        self.joint_state_publisher.publish(joint_state)
        
        # Publicar clock si está habilitado
        if self.use_sim_time:
            clock = Clock()
            clock.clock = self.get_clock().now().to_msg()
            self.clock_publisher.publish(clock)
    
    def check_connection(self):
        """
        Timer callback: verifica conexión con Isaac Sim.
        """
        if self.enable_isaac and not self.isaac_connected:
            self.get_logger().debug("Attempting to reconnect to Isaac Sim...")
            self._connect_isaac()


def main(args=None):
    """
    Punto de entrada principal del nodo.
    """
    # Inicializar ROS2
    rclpy.init(args=args)
    
    try:
        # Crear nodo
        node = CR20IsaacBridge()
        
        # Usar executor multi-threaded para mejor rendimiento
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        self.get_logger().info("Spinning node...")
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Limpieza
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

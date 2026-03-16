# Configuración Física del Robot CR20 - Referencia para Isaac Sim

Este documento contiene todos los parámetros de física y control del robot Dobot CR20, diseñados para ser utilizados como referencia al configurar la simulación en NVIDIA Isaac Sim.

---

## 1. Resumen de Parámetros

| Parámetro | Valor |
|-----------|-------|
| **Nombre del Robot** | cr20_robot |
| **Grados de Libertad** | 6 |
| **Tipo de articulaciones** | revolute (todas) |
| **Velocidad máxima articular** | 3.14 rad/s |
| **Aceleración máxima** | 5.0 rad/s² |
| **Masa total** | 29.0 kg |
| **Sistema de control** | ros2_control + JointTrajectoryController |

---

## 2. Límites de Movimiento por Articulación

### 2.1 Tabla Comparativa

| Joint | Tipo | Lower (rad) | Lower (°) | Upper (rad) | Upper (°) | Vel. Máx (rad/s) | Effort (Nm) |
|-------|------|-------------|-----------|-------------|-----------|------------------|-------------|
| joint1 | revolute | -6.28 | -360° | 6.28 | +360° | 3.14 | 150 |
| joint2 | revolute | -6.28 | -360° | 6.28 | +360° | 3.14 | 150 |
| joint3 | revolute | -2.9 | -166° | 2.9 | +166° | 3.14 | 150 |
| joint4 | revolute | -6.28 | -360° | 6.28 | +360° | 3.14 | 50 |
| joint5 | revolute | -6.28 | -360° | 6.28 | +360° | 3.14 | 50 |
| joint6 | revolute | -6.28 | -360° | 6.28 | +360° | 3.14 | 50 |

### 2.2 Notas Importantes

- **joint3** tiene un rango limitado (±2.9 rad ≈ ±166°) para evitar colisiones con la base
- **joint1-joint2** tienen mayor capacidad de esfuerzo (150 Nm) para manipular cargas pesadas
- **joint4-joint6** tienen menor capacidad (50 Nm) ya que son las articulaciones de la muñeca
- El factor de escala por defecto es **0.1** (10% de velocidad/aceleración) para movimientos seguros

---

## 3. Parámetros Físicos por Eslabón (Link)

### 3.1 Masas e Inercias

| Link | Masa (kg) | Ixx | Iyy | Izz | Centro de Masa (xyz) |
|------|-----------|-----|-----|-----|---------------------|
| base_link | 5.0 | 0.01 | 0.01 | 0.01 | -0.053, 0.090, -0.003 |
| Link1 | 8.0 | 0.05 | 0.05 | 0.02 | -0.015, -0.099, 1.041 |
| Link2 | 6.0 | 0.1 | 0.1 | 0.02 | -0.829, 0.0, 0.115 |
| Link3 | 4.0 | 0.05 | 0.05 | 0.01 | -0.746, 0.0, 0.076 |
| Link4 | 3.0 | 0.01 | 0.01 | 0.005 | 0.0, -0.076, 0.0 |
| Link5 | 2.0 | 0.005 | 0.005 | 0.003 | 0.0, 0.076, 0.0 |
| Link6 | 1.0 | 0.002 | 0.002 | 0.001 | 0.015, 0.659, -0.213 |
| **TOTAL** | **29.0** | - | - | - | - |

### 3.2 Distribución de Masa

```
Masa total: 29.0 kg

base_link ████████████████████ 5.0 kg (17.2%)
Link1     ████████████████████████████████ 8.0 kg (27.6%)
Link2     █████████████████████████ 6.0 kg (20.7%)
Link3     ██████████████████ 4.0 kg (13.8%)
Link4     ██████████████ 3.0 kg (10.3%)
Link5     ██████████ 2.0 kg (6.9%)
Link6     █████ 1.0 kg (3.4%)
```

---

## 4. Parámetros de Control (Dinámica)

### 4.1 Amortiguación y Fricción por Joint

| Joint | Damping (Ns/m) | Friction (Nm) | Notas |
|-------|-----------------|----------------|-------|
| joint1 | 10.0 | 0.5 | Motor principal - alto damping |
| joint2 | 10.0 | 0.5 | Motor principal - alto damping |
| joint3 | 8.0 | 0.4 | Motor principal |
| joint4 | 5.0 | 0.3 | Muñeca - damping medio |
| joint5 | 3.0 | 0.2 | Muñeca - bajo damping |
| joint6 | 2.0 | 0.1 | Muñeca - mínimo damping |

### 4.2 Configuración de ros2_control

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz (10ms per tick)

    cr20_group_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

cr20_group_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    command_interfaces:
      - position    # Control por posición
    state_interfaces:
      - position
      - velocity
```

### 4.3 Interfaces de Comando

| Interfaz | Tipo | Descripción |
|----------|------|-------------|
| command | position | Solo control de posición |
| state | position | Retroalimentación de posición |
| state | velocity | Retroalimentación de velocidad |

---

## 5. Configuración de MoveIt

### 5.1 Límites de Velocidad y Aceleración

```yaml
default_velocity_scaling_factor: 0.1    # 10% de velocidad máxima
default_acceleration_scaling_factor: 0.1 # 10% de aceleración máxima

joint_limits:
  joint1:
    has_velocity_limits: true
    max_velocity: 3.14      # rad/s
    has_acceleration_limits: true
    max_acceleration: 5.0  # rad/s²
  # ... (mismo para joint2-joint6)
```

### 5.2 Límites Cartesianos (Pilz Planner)

```yaml
cartesian_limits:
  max_trans_vel: 1.0        # m/s
  max_trans_acc: 2.25       # m/s²
  max_trans_dec: -5.0       # m/s² (deceleración)
  max_rot_vel: 1.57         # rad/s (90°/s)
```

### 5.3 Cinemática

```yaml
cr20_group:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

---

## 6. Posiciones Home y Estados

### 6.1 Posición Home (cero)

```
joint1: 0°    (0.0 rad)
joint2: 0°    (0.0 rad)
joint3: 0°    (0.0 rad)
joint4: 0°    (0.0 rad)
joint5: 0°    (0.0 rad)
joint6: 0°    (0.0 rad)
```

### 6.2 Posición Home (SRDF)

```
joint1: 0°     (0.0 rad)
joint2: 45.7°  (0.798 rad)
joint3: -109.2° (-1.907 rad)
joint4: -6.0°  (-0.104 rad)
joint5: 93.4°  (1.631 rad)
joint6: 0°     (0.0 rad)
```

---

## 7. Configuración para Isaac Sim

### 7.1 Parámetros a Configurar

| Parámetro | Valor CR20 | Notas |
|-----------|------------|-------|
| **Physics Engine** | PhysX | Recomendado |
| **Gravity** | 0, 0, -9.81 m/s² | Gravedad estándar |
| **Timestep** | 1/1000 s | Alta precisión |
| **Solver Type** | TGS | Temporal Gauss Seidel |

### 7.2 Articulaciones (Joints) para Isaac Sim

| Joint Name | Type | Axis | Position Limits | Velocity Limit | Effort Limit |
|------------|------|------|-----------------|----------------|--------------|
| joint1 | revolute | Z | [-6.28, 6.28] rad | 3.14 rad/s | 150 Nm |
| joint2 | revolute | Z | [-6.28, 6.28] rad | 3.14 rad/s | 150 Nm |
| joint3 | revolute | Z | [-2.9, 2.9] rad | 3.14 rad/s | 150 Nm |
| joint4 | revolute | Z | [-6.28, 6.28] rad | 3.14 rad/s | 50 Nm |
| joint5 | revolute | Z | [-6.28, 6.28] rad | 3.14 rad/s | 50 Nm |
| joint6 | revolute | Z | [-6.28, 6.28] rad | 3.14 rad/s | 50 Nm |

### 7.3 Rigid Bodies (Links) para Isaac Sim

| Link Name | Mass (kg) | Inertia (Ixx, Iyy, Izz) | Collision Mesh |
|-----------|-----------|-------------------------|----------------|
| base_link | 5.0 | (0.01, 0.01, 0.01) | base_link.STL |
| Link1 | 8.0 | (0.05, 0.05, 0.02) | Link1.STL |
| Link2 | 6.0 | (0.1, 0.1, 0.02) | Link2.STL |
| Link3 | 4.0 | (0.05, 0.05, 0.01) | Link3.STL |
| Link4 | 3.0 | (0.01, 0.01, 0.005) | Link4.STL |
| Link5 | 2.0 | (0.005, 0.005, 0.003) | Link5.STL |
| Link6 | 1.0 | (0.002, 0.002, 0.001) | Link6.STL |

### 7.4 Damping y Friction para Isaac Sim

| Joint | Damping | Friction | Notas |
|-------|---------|----------|-------|
| joint1 | 10.0 | 0.5 | Motor principal |
| joint2 | 10.0 | 0.5 | Motor principal |
| joint3 | 8.0 | 0.4 | Motor principal |
| joint4 | 5.0 | 0.3 | Muñeca |
| joint5 | 3.0 | 0.2 | Muñeca |
| joint6 | 2.0 | 0.1 | Muñeca |

### 7.5 Example Isaac Sim Joint Configuration (Python)

```python
# Configuración de articulación para Isaac Sim
joint_config = {
    "joint1": {
        "type": "revolute",
        "axis": [0, 0, 1],
        "position_limits": [-6.28, 6.28],  # rad
        "velocity_limit": 3.14,  # rad/s
        "effort_limit": 150.0,  # Nm
        "damping": 10.0,
        "friction": 0.5,
    },
    "joint2": {
        "type": "revolute",
        "axis": [0, 0, 1],
        "position_limits": [-6.28, 6.28],
        "velocity_limit": 3.14,
        "effort_limit": 150.0,
        "damping": 10.0,
        "friction": 0.5,
    },
    "joint3": {
        "type": "revolute",
        "axis": [0, 0, 1],
        "position_limits": [-2.9, 2.9],  # Limitado para evitar colisión
        "velocity_limit": 3.14,
        "effort_limit": 150.0,
        "damping": 8.0,
        "friction": 0.4,
    },
    "joint4": {
        "type": "revolute",
        "axis": [0, 0, 1],
        "position_limits": [-6.28, 6.28],
        "velocity_limit": 3.14,
        "effort_limit": 50.0,
        "damping": 5.0,
        "friction": 0.3,
    },
    "joint5": {
        "type": "revolute",
        "axis": [0, 0, 1],
        "position_limits": [-6.28, 6.28],
        "velocity_limit": 3.14,
        "effort_limit": 50.0,
        "damping": 3.0,
        "friction": 0.2,
    },
    "joint6": {
        "type": "revolute",
        "axis": [0, 0, 1],
        "position_limits": [-6.28, 6.28],
        "velocity_limit": 3.14,
        "effort_limit": 50.0,
        "damping": 2.0,
        "friction": 0.1,
    },
}

# Configuración de links (masas)
link_masses = {
    "base_link": 5.0,
    "Link1": 8.0,
    "Link2": 6.0,
    "Link3": 4.0,
    "Link4": 3.0,
    "Link5": 2.0,
    "Link6": 1.0,
}

# Inercias (Ixx, Iyy, Izz)
link_inertias = {
    "base_link": (0.01, 0.01, 0.01),
    "Link1": (0.05, 0.05, 0.02),
    "Link2": (0.1, 0.1, 0.02),
    "Link3": (0.05, 0.05, 0.01),
    "Link4": (0.01, 0.01, 0.005),
    "Link5": (0.005, 0.005, 0.003),
    "Link6": (0.002, 0.002, 0.001),
}
```

---

## 8. Archivos de Origen

| Archivo | Ruta |
|---------|------|
| URDF | `src/DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr20_robot.urdf` |
| Joint Limits | `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/config/joint_limits.yaml` |
| ros2_controllers | `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/config/ros2_controllers.yaml` |
| moveit_controllers | `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/config/moveit_controllers.yaml` |
| Kinematics | `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/config/kinematics.yaml` |
| SRDF | `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/config/cr20_robot.srdf` |
| ros2_control xacro | `src/DOBOT_6Axis_ROS2_V4/cr20_moveit/config/cr20_robot.ros2_control.xacro` |
| Meshes | `src/DOBOT_6Axis_ROS2_V4/cra_description/meshes/cr20/*.STL` |

---

## 9. Notas de Configuración

1. **Factor de escala**: El sistema usa 0.1 (10%) por defecto para movimientos seguros. En Isaac Sim, se puede usar 1.0 para movimientos a máxima velocidad.

2. **joint3 limitado**: El joint3 tiene un rango de ±2.9 rad (±166°) para evitar colisiones con la base del robot. Este límite es crítico.

3. **Control por posición**: El robot está configurado para control por posición únicamente (`command_interfaces: position`).

4. **Fricción decreciente**: La fricción disminuye desde joint1 hasta joint6, lo que refleja la reducción de tamaño y capacidad de los motores de la muñeca.

5. **Archivo USD existente**: El archivo `Modelo/CR20V_isaac.usd` ya contiene una versión del robot para Isaac Sim.

---

*Reporte generado automáticamente para referencia de configuración en Isaac Sim*

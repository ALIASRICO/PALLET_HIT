# Despaletizador Dobot CR20

Sistema de despaletización industrial automático con robot Dobot CR20, cámara Intel RealSense D435i y detección YOLO OBB. Detecta cajas de jugo (MANGO/MORA) en una paleta y las transfiere a una cinta transportadora usando una ventosa de vacío.

---

## 🤖 Descripción General

| Componente | Detalle |
|---|---|
| **Robot** | Dobot CR20 (6 ejes, 20 kg payload) |
| **Cámara** | Intel RealSense D435i |
| **Gripper** | Ventosa de vacío (salida digital DO1) |
| **Software** | ROS2 Jazzy, MoveIt2, YOLO OBB |
| **Modelo IA** | `kartonger_best.pt` (YOLOv8-OBB) |
| **Conexión robot** | TCP/IP → `192.168.5.1:29999/30003/30004` |

El robot recibe coordenadas 3D de las cajas desde la cámara, planifica trayectorias con MoveIt2 y ejecuta ciclos de pick-and-place de forma autónoma.

---

## 🏗️ Arquitectura del Sistema

```
[RealSense D435i] → [yolo_kartonger_detector] → /detections/kartonger
                                                          ↓
[Robot CR20] ←→ [dobot_bringup_v4] ←→ [action_move_server] ←→ [MoveIt2] ←→ [depalletizer]
                       ↓
               /joint_states_robot → joint_states bridge → /joint_states
```

El sistema opera con **5 terminales** en paralelo (ver sección [Ejecución](#-ejecución)).

**Ciclo de operación del depalletizer:**

```
APPROACH → DESCENSO → DO1 ON (ventosa) → LIFT → PLACE → DO1 OFF
```

---

## 📦 Paquetes ROS2

| Paquete | Tipo | Descripción |
|---|---|---|
| `dobot_bringup_v4` | C++ | Driver TCP del robot CR20 |
| `dobot_msgs_v4` | C++ | 2 mensajes + 121 servicios personalizados |
| `cr20_moveit` | Config | Configuración MoveIt2 del CR20 |
| `dobot_moveit` | Python | Nodos de movimiento: `depalletizer`, `action_move_server`, `scene_manager`, `calibration` |
| `dobot_camera` | Python | Nodos de visión: `vision_node` (AprilTags), `yolo_detector` (YOLO OBB) |
| `dobot_rviz` | Launch | Visualización RViz e Isaac Sim |
| `cra_description` | URDF | Modelo 3D del robot (URDF + meshes STL) |
| `gz_conveyorbelt` | Python | Plugin Gazebo de cinta transportadora |
| `pruebas_de_vision` | Python | Pruebas de detección YOLO con venv |

---

## 🔌 Nodos Principales

### dobot_bringup_ros2 (C++)
Driver del robot. Mantiene la conexión TCP/IP con el CR20.

| Dirección | Topic / Servicio |
|---|---|
| Publica | `/joint_states_robot`, `/dobot_msgs_v4/msg/ToolVectorActual`, `/dobot_msgs_v4/msg/RobotStatus` |
| Servicios | 121 servicios: `MovJ`, `MovL`, `ServoJ`, `ServoP`, `EnableRobot`, `DisableRobot`, `DOInstant`, `GetPose`, ... |

### joint_states (Python)
Bridge de estados de articulaciones.

| Dirección | Topic |
|---|---|
| Suscribe | `/joint_states_robot` |
| Publica | `/joint_states` |

### action_move_server (Python)
Action Server `FollowJointTrajectory`. Interpola waypoints de MoveIt2 y los envía al robot via `ServoJ` cada 100 ms.

### scene_manager (Python)
Gestiona objetos de colisión en MoveIt2. Lee la geometría del entorno desde `collision_config.json`.

### yolo_kartonger_detector (Python)
Detector YOLO OBB en tiempo real.

| Dirección | Topic | Contenido |
|---|---|---|
| Publica | `/detections/kartonger` | `[id, x, y, z, conf, w, h, angle]` |
| Publica | `/detections/image` | Imagen anotada |

### depalletizer (Python)
Nodo principal del sistema. Suscribe `/detections/kartonger` y `/joint_states`, ejecuta el ciclo completo de pick-and-place.

---

## 📡 Mensajes y Servicios

### Mensajes personalizados (`dobot_msgs_v4`)

| Mensaje | Descripción |
|---|---|
| `RobotStatus.msg` | Estado general del robot (errores, modo, habilitación) |
| `ToolVectorActual.msg` | Posición TCP actual en mm: `x, y, z, rx, ry, rz` |

### Servicios destacados

`MovJ`, `MovL`, `ServoJ`, `ServoP`, `EnableRobot`, `DisableRobot`, `DOInstant`, `GetPose` y 113 más definidos en `dobot_msgs_v4/srv/`.

---

## ⚙️ Configuración

### calibration_config.json

Calibración cámara-robot. Generado por `calibration_node`.

```json
{
  "affine_matrix_2x3": [[-0.982, -0.097, 186.6], [-0.097, 0.991, 1111.2]],
  "z_plane_cam":   { "a": -0.020, "b": 0.013, "c": 1905.3 },
  "z_plane_robot": { "a": 0.0,   "b": 0.0,   "c": -620.9 },
  "z_work_mm_fallback": -620.9
}
```

- `affine_matrix_2x3`: Transformación afín XY de coordenadas de cámara a coordenadas de robot.
- `z_plane_cam` / `z_plane_robot`: Planos de superficie para calcular la altura real del objeto.

### collision_config.json

Objetos de colisión del entorno. Generado por `collision_calibrator`.

```json
{
  "objects": {
    "suelo":               { "type": "floor",   "z_mm": -650.0 },
    "techo":               { "type": "ceiling", "z_mm": 1276.4 },
    "cinta_transportadora":{ "type": "box",     "corner1_mm": [...], "corner2_mm": [...] },
    "camara_soporte":      { "type": "pole",    "bottom_mm": [...], "width_mm": 80 },
    "pared":               { "type": "wall",    "point1_mm": [...], "point2_mm": [...] }
  },
  "place_position_m": [1.031, 0.349, 0.350]
}
```

- `place_position_m`: Posición de depósito en la cinta transportadora (metros, frame robot).

---

## 🛠️ Requisitos e Instalación

### Sistema operativo y middleware

- Ubuntu 24.04 LTS
- ROS2 Jazzy
- MoveIt2 para Jazzy

### Dependencias Python

```bash
pip install pyrealsense2 opencv-python ultralytics pupil-apriltags numpy
```

### Compilar el workspace

```bash
cd ~/dobot_ws
colcon build --symlink-install
source install/setup.bash
```

### Source obligatorio (en cada terminal)

```bash
source /opt/ros/jazzy/setup.bash
source ~/dobot_ws/install/setup.bash
```

---

## 🚀 Ejecución

Abrir 5 terminales. En cada una, hacer source antes de ejecutar.

```bash
source /opt/ros/jazzy/setup.bash
source ~/dobot_ws/install/setup.bash
```

### Terminal 1 — Driver del robot

```bash
export IP_address=192.168.5.1
export DOBOT_TYPE=cr20
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

### Terminal 2 — MoveIt2

```bash
export DOBOT_TYPE=cr20
ros2 launch cr20_moveit real_robot.launch.py
```

### Terminal 3 — Detector YOLO

```bash
~/dobot_ws/scripts/run_yolo.sh
```

### Terminal 4 — Action Move Server

```bash
export IP_address=192.168.5.1
ros2 run dobot_moveit action_move_server
```

### Terminal 5 — Despaletizador

```bash
ros2 run dobot_moveit depalletizer
```

> Ver [ejecutar.md](ejecutar.md) para referencia rápida de comandos.

---

## 🎯 Calibración

### Calibración cámara-robot (XY + Z)

Requiere AprilTags visibles en al menos 4 puntos del espacio de trabajo.

```bash
ros2 run dobot_moveit calibration_node
```

El nodo guía el proceso interactivo y guarda el resultado en `calibration_config.json`.

### Calibración de colisiones

Guía interactiva para definir los 7 objetos del entorno (suelo, techo, cinta, soporte de cámara, pared, etc.).

```bash
ros2 run dobot_moveit collision_calibrator
```

Guarda el resultado en `collision_config.json`.

> Ver [CR20_PHYSICS_CONFIG.md](CR20_PHYSICS_CONFIG.md) para parámetros de simulación en Isaac Sim.

---

## 📁 Estructura de Directorios

```
dobot_ws/
├── src/
│   ├── DOBOT_6Axis_ROS2_V4/
│   │   ├── dobot_bringup_v4/       # Driver C++ del robot
│   │   ├── dobot_msgs_v4/          # Mensajes y servicios
│   │   ├── cr20_moveit/            # Configuración MoveIt2
│   │   ├── dobot_moveit/           # Nodos de movimiento y calibración
│   │   ├── dobot_camera/           # Nodos de visión (AprilTags + YOLO)
│   │   ├── dobot_rviz/             # Visualización RViz
│   │   └── cra_description/        # Modelo URDF + meshes STL
│   ├── gz_conveyorbelt/            # Plugin Gazebo de cinta transportadora
│   └── pruebas_de_vision/          # Pruebas de detección YOLO con venv
├── calibration_config.json         # Calibración cámara-robot
├── collision_config.json           # Objetos de colisión MoveIt2
├── yolo_training/                  # Modelo YOLO (kartonger_best.pt)
├── scripts/
│   └── run_yolo.sh                 # Script de lanzamiento del detector
├── ejecutar.md                     # Referencia rápida de comandos
└── CR20_PHYSICS_CONFIG.md          # Parámetros para Isaac Sim
```

> Ver [src/pruebas_de_vision/README.md](src/pruebas_de_vision/README.md) para detalles de entrenamiento YOLO y uso del dataset Kartonger.

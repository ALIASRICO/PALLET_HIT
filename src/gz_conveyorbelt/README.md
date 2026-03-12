# Banda Transportadora - "The Stationary Pusher"

## Descripción

Sistema de banda transportadora para simulación en Gazebo Sim 8 (Harmonic) con ROS2 Jazzy, diseñado para integración con brazo robótico DOBOT CR20V.

### Arquitectura: Doble Capa

```
┌─────────────────────────────────────────────────────────────┐
│  conveyor_chassis (ESTÁTICO - Pose: 0,0,0 SIEMPRE)         │
│  ├── Visual de banda con textura                           │
│  ├── Rieles laterales                                      │
│  └── Marcadores de pick/place                              │
│                                                             │
│  ════════════════════════════════════════════════════════  │
│                                                             │
│  conveyor_surface (CINEMÁTICO - INVISIBLE)                 │
│  ├── Sin visual                                            │
│  ├── Collision: 2m × 0.5m × 0.001m (placa invisible)       │
│  ├── Pose reseteada a (0,0,0.125) cada 50ms                │
│  └── Velocidad: 0.1 m/s via VelocityControl                │
└─────────────────────────────────────────────────────────────┘
```

## Ventajas para Brazo Robótico

| Aspecto | Garantía |
|---------|----------|
| **Referencia espacial** | El `chassis` estático SIEMPRE está en (0,0,0) |
| **Zona de colisión fija** | La superficie invisible tiene 2m de ancho, centrada en origen |
| **Sin teletransporte** | El reset de pose de un link cinemático no perturba objetos |
| **Predicción de contacto** | El brazo sabe exactamente dónde estará la superficie de contacto |
| **Visualización correcta** | El operador ve la banda en su posición real |

## Archivos

```
src/gz_conveyorbelt/
├── worlds/
│   └── conveyor_test.sdf              # Mundo principal (SDF unificado)
├── gz_conveyorbelt/
│   └── conveyor_logic_controller.py   # Nodo ROS2 maestro
├── config/
│   └── bridge_scripted.yaml           # Configuración del bridge
└── launch/
    └── conveyor_test.launch.py        # Launch file
```

## Uso

### Opción 1: Launch completo (Recomendado)

```bash
# Terminal 1: Lanzar simulación completa
ros2 launch gz_conveyorbelt conveyor_test.launch.py

# Terminal 2: Iniciar banda
ros2 service call /conveyor/start std_srvs/srv/SetBool "{data: true}"
```

### Opción 2: Manual paso a paso

```bash
# Terminal 1: Lanzar Gazebo
gz sim src/gz_conveyorbelt/worlds/conveyor_test.sdf -r

# Terminal 2: Lanzar bridge
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=src/gz_conveyorbelt/config/bridge_scripted.yaml

# Terminal 3: Lanzar nodo controlador
ros2 run gz_conveyorbelt conveyor_logic_controller.py --ros-args -p belt_speed:=0.1

# Terminal 4: Iniciar banda
ros2 service call /conveyor/start std_srvs/srv/SetBool "{data: true}"
```

### Opción 3: Solo Gazebo (sin ROS2)

```bash
# Terminal 1: Lanzar Gazebo
gz sim src/gz_conveyorbelt/worlds/conveyor_test.sdf -r

# Terminal 2: Control manual via tópico Gazebo
gz topic -t /model/conveyor_surface/link/surface/cmd_vel \
  -m gz.msgs.Twist \
  -p "linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}"
```

## Comandos de Control

### Via ROS2 Services

```bash
# Iniciar banda
ros2 service call /conveyor/start std_srvs/srv/SetBool "{data: true}"

# Detener banda
ros2 service call /conveyor/stop std_srvs/srv/SetBool "{data: true}"

# Cambiar velocidad
ros2 service call /conveyor/set_speed example_interfaces/srv/SetDouble "{data: 0.2}"
```

### Via Gazebo Topics

```bash
# Enviar velocidad
gz topic -t /model/conveyor_surface/link/surface/cmd_vel \
  -m gz.msgs.Twist \
  -p "linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}"

# Resetear pose
gz topic -t /model/conveyor_surface/link/surface/pose \
  -m gz.msgs.Pose \
  -p "position: {x: 0, y: 0, z: 0.125}, orientation: {w: 1, x: 0, y: 0, z: 0}"
```

## Coordenadas de Pick & Place

La estructura base define posiciones fijas:

| Punto | Coordenadas (X, Y, Z) | Descripción |
|-------|----------------------|-------------|
| Pick position | (-0.5, 0, 0.25) | Marcador amarillo |
| Place position | (0.5, 0, 0.25) | Marcador verde |
| Belt surface | (0, 0, 0.125) | Superficie de contacto invisible |

## Flujo de Trabajo con DOBOT CR20V

1. La banda transporta cajas desde X < 0 hacia X > 0
2. El brazo espera en `pick_position` (-0.5, 0, 0.25)
3. Cuando una caja llega al punto de pick, el brazo la agarra
4. El brazo mueve la caja al `place_position` (0.5, 0, 0.25)
5. Repetir

**Importante**: La superficie de la banda SIEMPRE está en la misma posición, por lo que el brazo puede calcular trayectorias sin preocuparse por el movimiento de la geometría.

## Parámetros de Fricción

La fricción alta garantiza que los objetos se muevan con la banda:

```xml
<friction>
  <ode>
    <mu>1000.0</mu>
    <mu2>1000.0</mu2>
    <slip1>0.0</slip1>
    <slip2>0.0</slip2>
  </ode>
  <torsional>
    <coefficient>100.0</coefficient>
  </torsional>
</friction>
```

## Build

```bash
cd ~/dobot_ws
colcon build --packages-select gz_conveyorbelt --symlink-install
source install/setup.bash
```

## Requisitos

- Ubuntu 24.04
- ROS2 Jazzy Jalisco
- Gazebo Sim 8 (Harmonic)

## Troubleshooting

### La banda no mueve las cajas

1. Verificar que el link `surface` tenga `<kinematic>true</kinematic>`
2. Verificar que la velocidad se esté publicando
3. Verificar que el reset de pose esté funcionando

```bash
# Verificar velocidad
gz topic -e -t /model/conveyor_surface/link/surface/cmd_vel

# Verificar pose
gz topic -e -t /model/conveyor_surface/link/surface/pose
```

### La visual de la banda se mueve

Esto no debería ocurrir con el diseño actual. Si pasa:

1. Verificar que `conveyor_chassis` tenga `<static>true</static>`
2. Verificar que `conveyor_surface` NO tenga visual
3. Verificar que el reset de pose esté funcionando

### El nodo controlador no inicia

1. Verificar que el bridge esté corriendo
2. Verificar que los tópicos existan
3. Revisar logs: `ros2 topic echo /rosout`

## Licencia

MIT

# Pruebas de Visión - YOLOv8-OBB para DOBOT CR20V

Sistema de detección de objetos usando YOLOv8-OBB (Oriented Bounding Box) para el robot DOBOT CR20V.

## 📦 Estructura

```
pruebas_de_vision/
├── pruebas_de_vision/
│   ├── camera_viewer.py      # Visualizador simple de cámara Gazebo
│   ├── train_yolo.py         # Script de entrenamiento YOLO
│   └── yolo_detector.py      # Nodo ROS2 de detección
├── launch/
│   ├── camera_test.launch.py         # Launch solo de cámara Gazebo
│   ├── yolo_detection.launch.py      # Launch completo (Gazebo + detector)
│   └── yolo_detection_venv.launch.py # Launch con entorno virtual
└── worlds/
    └── camera_test.sdf       # Mundo Gazebo con cámara
```

## 🔧 Instalación

### 1. Crear Entorno Virtual (YA HECHO)

```bash
cd ~/dobot_ws
python3 -m venv yolo_venv
```

### 2. Instalar Dependencias en el Venv

```bash
~/dobot_ws/yolo_venv/bin/pip install ultralytics roboflow opencv-python numpy
```

### 3. Compilar Paquete ROS2

```bash
cd ~/dobot_ws
colcon build --packages-select pruebas_de_vision --symlink-install
source install/setup.bash
```

## 🚀 Uso

### Visualizar cámara (sin detección)

```bash
# Lanzar Gazebo con cámara
ros2 launch pruebas_de_vision camera_test.launch.py

# En otra terminal, ver el feed de la cámara
source ~/dobot_ws/install/setup.bash
ros2 run pruebas_de_vision camera_viewer
```

### Opción A: Comandos Manuales

#### Entrenar Modelo

```bash
# Ejecutar con el entorno virtual
~/dobot_ws/yolo_venv/bin/python3 ~/dobot_ws/src/pruebas_de_vision/pruebas_de_vision/train_yolo.py
```

#### Ejecutar Detección con Gazebo

```bash
# Terminal 1: Source ROS2 y lanzar Gazebo
source ~/dobot_ws/install/setup.bash
ros2 launch pruebas_de_vision camera_test.launch.py

# Terminal 2: Ejecutar detector con venv
source ~/dobot_ws/install/setup.bash
~/dobot_ws/yolo_venv/bin/python3 ~/dobot_ws/src/pruebas_de_vision/pruebas_de_vision/yolo_detector.py
```

### Opción B: Launch File Integrado

```bash
# Lanza Gazebo + bridge + detector en un solo comando
source ~/dobot_ws/install/setup.bash
ros2 launch pruebas_de_vision yolo_detection.launch.py

# Con entorno virtual (si ultralytics no está en el sistema)
ros2 launch pruebas_de_vision yolo_detection_venv.launch.py
```

## 📡 Topics

### Publicados por el Detector

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/vision/detections/image` | sensor_msgs/Image | Imagen con detecciones anotadas |
| `/vision/detections/poses` | geometry_msgs/PoseArray | Poses 3D de objetos detectados (z=0, orientación desde ángulo OBB) |
| `/vision/detections/bboxes` | vision_msgs/Detection2DArray | Bounding boxes orientados |

### Suscritos

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | Imagen de cámara Gazebo |

## ⚙️ Parámetros

El detector acepta los siguientes parámetros:

```bash
# Línea de comandos
~/dobot_ws/yolo_venv/bin/python3 yolo_detector.py \
    --model /path/to/best.pt \
    --confidence 0.5 \
    --iou 0.5 \
    --image-topic /camera/color/image_raw \
    --show-window
```

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `--model` | yolov8n-obb.pt | Ruta al modelo YOLO |
| `--confidence` | 0.25 | Umbral de confianza mínima |
| `--iou` | 0.5 | Umbral IOU para NMS |
| `--image-topic` | /camera/color/image_raw | Topic de imagen |
| `--show-window` | True | Mostrar ventana OpenCV |
| `--no-window` | - | Desactivar ventana |

## 🎯 Dataset

El modelo se entrena con el dataset **Kartonger** de Roboflow:
- **Workspace**: jugos
- **Project**: kartonger-8b52f
- **Clases**: Cajas de cartón

## 📁 Modelos Entrenados

Después del entrenamiento, los modelos se guardan en:

```
~/runs/obb/kartonger_yolov8l/
├── weights/
│   ├── best.pt      # Mejor modelo (usar este)
│   ├── last.pt      # Última época
│   └── best.onnx    # Exportado a ONNX
├── results.csv      # Métricas de entrenamiento
└── results.png      # Gráficas de rendimiento
```

> **Nota:** Dependiendo de dónde se ejecute `train_yolo.py`, la ruta real puede quedar como
> `~/runs/obb/runs/obb/kartonger_yolov8l/` (con el subdirectorio duplicado). El detector
> busca en ambas rutas automáticamente.

## 🔍 Verificar Instalación

```bash
# Verificar que el entorno virtual tiene las dependencias
~/dobot_ws/yolo_venv/bin/python3 -c "from ultralytics import YOLO; print('✅ YOLO OK')"

# Verificar que ROS2 ve el paquete
source ~/dobot_ws/install/setup.bash
ros2 pkg list | grep pruebas_de_vision
```

## ⚠️ Notas Importantes

1. **NO instales ultralytics en el sistema** - Usa siempre el entorno virtual
2. **El entorno virtual está en** `~/dobot_ws/yolo_venv`
3. **COLCON_IGNORE** está configurado para que colcon ignore el venv
4. **Source ROS2** antes de usar cualquier comando ros2

## 🐛 Troubleshooting

### Error: "ultralytics not found"

```bash
# Verificar instalación en venv
~/dobot_ws/yolo_venv/bin/pip list | grep ultralytics

# Si no está, instalar
~/dobot_ws/yolo_venv/bin/pip install ultralytics
```

### Error: "No module named 'rclpy'"

El detector necesita ROS2. Asegúrate de hacer source:

```bash
source ~/dobot_ws/install/setup.bash
```

### Gazebo no muestra cámara

```bash
# Verificar que el bridge está funcionando
ros2 topic list | grep camera

# Verificar imágenes
ros2 topic hz /camera/color/image_raw
```

## 📊 Métricas de Entrenamiento

El entrenamiento robusto usa:
- **Épocas**: 100 (con early stopping)
- **Batch size**: 16
- **Optimizer**: AdamW
- **Augmentación**: Mosaic, MixUp, Copy-Paste
- **Learning rate**: 0.001

Tiempo estimado: 2-4 horas en GPU, 8-12 horas en CPU.

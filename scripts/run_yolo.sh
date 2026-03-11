#!/bin/bash
#═══════════════════════════════════════════════════════
# Ejecutar YOLO Detector con entorno virtual
#═══════════════════════════════════════════════════════

echo "🔧 Configurando entorno..."

# ROS2
source /opt/ros/jazzy/setup.bash
source ~/dobot_ws/install/setup.bash

# Entorno virtual YOLO
source ~/dobot_ws/yolo_training/venv/bin/activate

echo "🚀 Iniciando YOLO Detector..."
python3 ~/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_camera/dobot_camera/yolo_detector.py "$@"

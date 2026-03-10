#!/usr/bin/env python3
"""
================================================================
Detector YOLO para Jugos - Despaletizador
================================================================
Detecta jugos de caja usando YOLOv8 y cámara RealSense D435i.
Publica las posiciones en coordenadas de cámara (mm).

Publica:
  - /detections/jugos (Float32MultiArray) - [id, x_mm, y_mm, z_mm, conf, w_px, h_px, juice_type]
  - /detections/image (Image) - Imagen con detecciones dibujadas
================================================================
"""

import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
import os
from pathlib import Path

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
from dobot_camera.juice_logic import (
    associate_cara_f_with_hit,
    JUICE_TYPE_MANGO, JUICE_TYPE_MORA, JUICE_TYPE_UNKNOWN,
    CLASS_ID_CARA_F, CLASS_ID_HIT_MANGO, CLASS_ID_HIT_MORA,
)

# YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("⚠️ ultralytics no instalado: pip install ultralytics")


class YOLOJuiceDetector(Node):
    def __init__(self):
        super().__init__('yolo_juice_detector')
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('  DETECTOR YOLO - JUGOS DE CAJA')
        self.get_logger().info('=' * 50)

        # Parámetros
        self.declare_parameter('model_path', os.path.expanduser('/home/iudc/dobot_ws/yolo_training/models/jugos_best.pt'))
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('show_window', True)
        
        self.model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value
        self.show_window = self.get_parameter('show_window').value

        # Configuración cámara
        self.WIDTH = 640
        self.HEIGHT = 480
        self.MIN_DEPTH = 0.3
        self.MAX_DEPTH = 3.0

        # Cargar modelo YOLO
        self.model = None
        if YOLO_AVAILABLE:
            if os.path.exists(self.model_path):
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'✅ Modelo cargado: {self.model_path}')
            else:
                self.get_logger().error(f'❌ Modelo no encontrado: {self.model_path}')
                # Intentar modelo genérico
                self.model = YOLO('yolov8n.pt')
                self.get_logger().warn('⚠️ Usando modelo genérico yolov8n.pt')
        else:
            self.get_logger().error('❌ YOLO no disponible')

        # Inicializar RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 30)

        # Filtros de profundidad
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.hole_filter = rs.hole_filling_filter()
        self.align = rs.align(rs.stream.color)

        # Iniciar cámara
        try:
            self.profile = self.pipeline.start(self.config)
            self.get_logger().info('✅ Cámara RealSense iniciada')
            
            # Warmup
            for _ in range(30):
                self.pipeline.wait_for_frames()
            
            # Obtener intrínsecos
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            self.intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            
        except Exception as e:
            self.get_logger().error(f'❌ Error iniciando cámara: {e}')
            raise

        # Publicadores
        self.detections_pub = self.create_publisher(
            Float32MultiArray,
            '/detections/jugos',
            10
        )
        
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(
            Image,
            '/detections/image',
            10
        )

        # Ventana
        if self.show_window:
            cv2.namedWindow('YOLO - Detección de Jugos', cv2.WINDOW_AUTOSIZE)

        # Timer principal
        self.timer = self.create_timer(0.05, self.detect_loop)  # 20 FPS
        
        self.get_logger().info('✅ Detector iniciado')
        self.get_logger().info('   Topic: /detections/jugos')

    def get_depth_at_point(self, depth_frame, cx, cy, radius=3):
        """Obtiene profundidad promedio en un área"""
        depths = []
        for dy in range(-radius, radius+1):
            for dx in range(-radius, radius+1):
                x = max(0, min(cx + dx, self.WIDTH - 1))
                y = max(0, min(cy + dy, self.HEIGHT - 1))
                d = depth_frame.get_distance(x, y)
                if self.MIN_DEPTH < d < self.MAX_DEPTH:
                    depths.append(d)
        
        if len(depths) >= 3:
            return float(np.median(depths))
        return 0.0

    def detect_loop(self):
        """Bucle principal de detección"""
        if self.model is None:
            return

        try:
            # Obtener frames
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned = self.align.process(frames)
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()

            if not depth_frame or not color_frame:
                return

            # Aplicar filtros
            depth_filtered = self.spatial_filter.process(depth_frame)
            depth_filtered = self.temporal_filter.process(depth_filtered)
            depth_filtered = self.hole_filter.process(depth_filtered)
            depth_frame = depth_filtered.as_depth_frame()

            # Convertir a OpenCV
            color_image = np.asanyarray(color_frame.get_data())
            display = color_image.copy()

            # Detectar con YOLO
            results = self.model.predict(
                source=color_image,
                conf=self.confidence,
                verbose=False
            )

            # Procesar detecciones
            detections_data = []

            if len(results) > 0 and results[0].obb is not None and len(results[0].obb) > 0:
                cara_f_raw = []   # cara_F dets (class_id==2): pick targets
                hit_raw = []      # HIT dets (class_id==0,1): juice type labels

                for obb in results[0].obb:
                    xywhr = obb.xywhr[0].cpu().numpy()
                    cx_f, cy_f, w_f, h_f, angle = xywhr
                    cx_px = int(cx_f)
                    cy_px = int(cy_f)
                    conf = float(obb.conf)
                    cls_id = int(obb.cls)

                    # Depth + 3D projection (unchanged pipeline)
                    z = self.get_depth_at_point(depth_frame, cx_px, cy_px)
                    if z <= 0:
                        continue

                    point = rs.rs2_deproject_pixel_to_point(
                        self.intrinsics,
                        [float(cx_px), float(cy_px)],
                        z
                    )
                    x_mm = point[0] * 1000
                    y_mm = point[1] * 1000
                    z_mm = z * 1000

                    # Draw ALL detections for visualization
                    if cls_id == CLASS_ID_CARA_F:
                        vis_color = (0, 255, 0)    # Green
                        vis_label = f'cara_F ({conf:.0%})'
                    elif cls_id == CLASS_ID_HIT_MANGO:
                        vis_color = (0, 165, 255)  # Orange
                        vis_label = f'MANGO ({conf:.0%})'
                    else:
                        vis_color = (255, 0, 255)  # Magenta
                        vis_label = f'MORA ({conf:.0%})'

                    # Draw OBB rotated rectangle
                    angle_deg = math.degrees(angle)
                    rect = ((cx_f, cy_f), (w_f, h_f), angle_deg)
                    box_pts = cv2.boxPoints(rect)
                    box_pts = box_pts.astype(int)
                    cv2.polylines(display, [box_pts], True, vis_color, 2)
                    cv2.circle(display, (cx_px, cy_px), 5, (0, 0, 255), -1)
                    cv2.putText(display, vis_label, (cx_px, cy_px - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, vis_color, 2)
                    cv2.putText(display, f'Z:{z_mm:.0f}mm', (cx_px, cy_px + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, vis_color, 1)

                    det_dict = {
                        'cx': cx_f, 'cy': cy_f,
                        'x_mm': x_mm, 'y_mm': y_mm, 'z_mm': z_mm,
                        'conf': conf, 'w_px': w_f, 'h_px': h_f,
                        'class_id': cls_id,
                    }
                    if cls_id == CLASS_ID_CARA_F:
                        cara_f_raw.append(det_dict)
                    elif cls_id in (CLASS_ID_HIT_MANGO, CLASS_ID_HIT_MORA):
                        hit_raw.append(det_dict)

                # Associate cara_F with juice type
                cara_f_associated = associate_cara_f_with_hit(cara_f_raw, hit_raw)

                # Build 8-field output: [id, x_mm, y_mm, z_mm, conf, w_px, h_px, juice_type]
                for pub_id, det in enumerate(cara_f_associated):
                    juice_type = det['juice_type']
                    if juice_type == JUICE_TYPE_MANGO:
                        overlay_color = (0, 255, 0)    # Green
                        type_label = 'MANGO'
                    elif juice_type == JUICE_TYPE_MORA:
                        overlay_color = (255, 0, 128)  # Purple
                        type_label = 'MORA'
                    else:
                        overlay_color = (0, 255, 255)  # Yellow
                        type_label = '???'

                    cx_px2 = int(det['cx'])
                    cy_px2 = int(det['cy'])
                    cv2.putText(display, f'{type_label} #{pub_id}',
                                (cx_px2, cy_px2 - 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, overlay_color, 2)

                    detections_data.extend([
                        float(pub_id),       # ID
                        det['x_mm'],         # X cam mm
                        det['y_mm'],         # Y cam mm
                        det['z_mm'],         # Z cam mm (depth)
                        det['conf'],         # Confidence
                        float(det['w_px']),  # Width px
                        float(det['h_px']),  # Height px
                        juice_type,          # 0.0=MANGO, 1.0=MORA, -1.0=UNKNOWN
                    ])

            # Publicar detecciones
            if detections_data:
                msg = Float32MultiArray()
                msg.data = detections_data
                self.detections_pub.publish(msg)
                
            # Info en pantalla
            num_detections = len(detections_data) // 8
            cv2.putText(display, f'Jugos detectados: {num_detections}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Publicar imagen
            img_msg = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            self.image_pub.publish(img_msg)

            # Mostrar ventana
            if self.show_window:
                cv2.imshow('YOLO - Detección de Jugos', display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('Cerrando...')
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        try:
            self.pipeline.stop()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = YOLOJuiceDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

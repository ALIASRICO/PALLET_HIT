#!/usr/bin/env python3
"""
Entrenamiento YOLOv8-OBB para jugos
Dataset: Roboflow jugos_hit v4
Todo queda en ~/dobot_ws/yolo_training/
"""

import os
import sys
from pathlib import Path

# ── Rutas base — todo dentro del workspace ────────────────
BASE    = Path(__file__).parent
DATASET = BASE / 'dataset'
RUNS    = BASE / 'runs'
MODELS  = BASE / 'models'

RUNS.mkdir(exist_ok=True)
MODELS.mkdir(exist_ok=True)

# ── Forzar que ultralytics guarde en nuestras rutas ───────
os.environ['YOLO_CONFIG_DIR'] = str(BASE / 'ultralytics_config')

from roboflow import Roboflow
from ultralytics import YOLO

# ── 1. Usar dataset ya descargado ──────────────────────────────────
DATA_YAML = BASE / 'dataset' / 'jugos_hit_v4' / 'data.yaml'
if not DATA_YAML.exists():
    print(f'❌ No se encontró: {DATA_YAML}')
    sys.exit(1)
print(f'✅ Dataset encontrado: {DATA_YAML}')

# ── 2. Encontrar el data.yaml ─────────────────────────────
yaml_files = list(DATASET.rglob('data.yaml'))
if not yaml_files:
    print('❌ No se encontró data.yaml en el dataset')
    sys.exit(1)
DATA_YAML = yaml_files[0]
print(f'📄 data.yaml: {DATA_YAML}')

# ── 3. Entrenar ───────────────────────────────────────────
print('\n🚀 Iniciando entrenamiento...')
model = YOLO('yolov8l-obb.pt')   # Largue-OBB

results = model.train(
    data    = str(DATA_YAML),
    epochs  = 150,
    imgsz   = 640,
    batch   = 8,
    device  = 0,               # GPU 0
    project = str(RUNS),
    name    = 'jugos_obb',
    exist_ok= True,
    plots   = True,
    save    = True,
    patience= 30,
    workers = 4,
)

# ── 4. Copiar mejor modelo a models/ ─────────────────────
best_pt  = Path(results.save_dir) / 'weights' / 'best.pt'
last_pt  = Path(results.save_dir) / 'weights' / 'last.pt'
dest_best = MODELS / 'jugos_best.pt'
dest_last = MODELS / 'jugos_last.pt'

if best_pt.exists():
    import shutil
    shutil.copy2(best_pt, dest_best)
    shutil.copy2(last_pt, dest_last)
    print(f'\n✅ Modelos guardados:')
    print(f'   {dest_best}')
    print(f'   {dest_last}')
else:
    print('⚠️ No se encontró best.pt')

print('\n🎉 Entrenamiento completado')
print(f'   Resultados: {results.save_dir}')
print(f'   Mejor modelo: {dest_best}')

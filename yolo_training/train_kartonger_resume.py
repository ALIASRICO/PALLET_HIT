#!/usr/bin/env python3
"""
Continue kartonger-1 YOLOv8L-OBB training: load best weights, train 200 more epochs.
Same hyperparameters as original run.
"""
import os
import sys
import shutil
from pathlib import Path

BASE = Path(__file__).parent
os.environ['YOLO_CONFIG_DIR'] = str(BASE / 'ultralytics_config')

from ultralytics import YOLO

# Load from best weights of previous training
WEIGHTS = Path('/home/iudc/runs/obb/runs/obb/kartonger_yolov8l/weights/best.pt')
if not WEIGHTS.exists():
    print(f'❌ Weights not found: {WEIGHTS}')
    sys.exit(1)

print(f'✅ Loading weights from: {WEIGHTS}')
model = YOLO(str(WEIGHTS))

# Train 200 more epochs with same hyperparameters as original
results = model.train(
    data='/home/iudc/dobot_ws/kartonger-1/data.yaml',
    task='obb',
    epochs=200,
    batch=16,
    imgsz=640,
    device=0,
    workers=8,
    optimizer='AdamW',
    lr0=0.001,
    lrf=0.01,
    momentum=0.937,
    weight_decay=0.0005,
    warmup_epochs=3.0,
    warmup_momentum=0.8,
    warmup_bias_lr=0.1,
    degrees=45.0,
    translate=0.1,
    scale=0.5,
    shear=0.0,
    perspective=0.0005,
    flipud=0.0,
    fliplr=0.5,
    mosaic=1.0,
    mixup=0.1,
    copy_paste=0.1,
    erasing=0.4,
    close_mosaic=10,
    patience=50,
    save_period=10,
    seed=42,
    deterministic=True,
    amp=True,
    project='runs/obb',
    name='kartonger_yolov8l_v2',
    exist_ok=True,
    plots=True,
    box=7.5,
    cls=0.5,
    dfl=1.5,
    angle=1.0,
    pretrained=True,
)

# Copy best model to models/
MODELS = BASE / 'models'
MODELS.mkdir(exist_ok=True)

save_dir = Path(results.save_dir) if hasattr(results, 'save_dir') and results.save_dir else Path('runs/obb/kartonger_yolov8l_v2')
best_src = save_dir / 'weights' / 'best.pt'
last_src = save_dir / 'weights' / 'last.pt'

if best_src.exists():
    shutil.copy2(best_src, MODELS / 'kartonger_best.pt')
    print(f'✅ kartonger_best.pt saved to {MODELS}')
else:
    print(f'⚠️ best.pt not found at {best_src}')

if last_src.exists():
    shutil.copy2(last_src, MODELS / 'kartonger_last.pt')
    print(f'✅ kartonger_last.pt saved to {MODELS}')
else:
    print(f'⚠️ last.pt not found at {last_src}')

print('🎉 Training complete')
print(f'   Results dir: {save_dir}')

import numpy as np

# Puntos de calibración (robot_x, robot_y, robot_z)
points = [
    [759.62,  1251.52, -505.7053],
    [263.86,  1216.37, -505.0],
    [782.72,  826.52,  -508.6789],
    [287.40,  787.30,  -505.0002],
]

pts = np.array(points)
X = pts[:, 0]
Y = pts[:, 1]
Z = pts[:, 2]

# Mínimos cuadrados: z = a*x + b*y + c
A = np.column_stack([X, Y, np.ones(len(X))])
result, _, _, _ = np.linalg.lstsq(A, Z, rcond=None)
a, b, c = result

print(f'"a": {a},')
print(f'"b": {b},')
print(f'"c": {c}')

# Verificar error por punto
for i, (x, y, z_real) in enumerate(points):
    z_pred = a*x + b*y + c
    print(f'Punto {i}: real={z_real:.4f} pred={z_pred:.4f} error={z_pred-z_real:.4f}mm')
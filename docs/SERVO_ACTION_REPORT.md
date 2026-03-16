# Informe Comparativo: `servo_action/` vs `action_move_server.py` + `joint_states.py`

**Fecha:** 12 de marzo de 2026
**Objetivo:** Determinar si el paquete `servo_action/` puede eliminarse de forma segura del workspace.

---

## 1. Resumen Ejecutivo

El paquete `servo_action/` es un paquete ROS 2 de tipo **cliente exclusivamente**, que contiene únicamente dos scripts Python funcionales (`action_move_client.py` y `Joint_Position.py`) con un total aproximado de **84 líneas de código**. Fue creado como herramienta de prueba/desarrollo para interactuar con el servidor de acciones `FollowJointTrajectory`, pero nunca evolucionó a código de producción.

En contraste, los archivos `action_move_server.py` (107 líneas) y `joint_states.py` (40 líneas) dentro del paquete `dobot_moveit` constituyen la implementación **completa y funcional** del servidor de acciones y la republicación de estados articulares. Estos archivos reemplazan y superan toda la funcionalidad que `servo_action/` pretendía ofrecer, con mejor manejo de errores, arquitectura multi-hilo y validación de datos.

Tras un análisis exhaustivo del código fuente y una búsqueda de dependencias en todo el workspace, **no se encontró ningún paquete, nodo o archivo externo que dependa de `servo_action/`**. Las únicas referencias a "servo_action" están dentro del propio paquete (su `setup.py`, `package.xml` y comentarios internos). Por lo tanto, su eliminación no afectaría a ningún componente del sistema.

---

## 2. Tabla Comparativa de Funcionalidad

| Característica | `servo_action/` | `action_move_server.py` + `joint_states.py` |
|---|---|---|
| **Tipo de componente** | Cliente de prueba | Servidor de producción |
| **Líneas de código total** | ~84 líneas (2 archivos) | ~147 líneas (2 archivos) |
| **Servidor FollowJointTrajectory** | ❌ No incluye | ✅ Completo (`action_move_server.py`) |
| **Cliente FollowJointTrajectory** | ✅ Básico, solo pruebas | ❌ No incluye (MoveIt lo proporciona) |
| **Republicación de joint_states** | ❌ Roto (variable indefinida) | ✅ Funcional (`joint_states.py`) |
| **Multi-threaded executor** | ❌ No | ✅ Sí (`MultiThreadedExecutor`, 3 hilos) |
| **Habilitación del robot** | ❌ No | ✅ `EnableRobot` service client |
| **Comando ServoJ** | ❌ No (solo envía goals) | ✅ Envío directo de comandos `ServoJ` |
| **Conversión rad→grados** | ❌ No | ✅ Sí (180 × rad / π) |
| **Validación de datos** | ❌ Ninguna | ✅ Validación de longitud de posiciones |
| **Manejo de timeouts** | ❌ No | ✅ Timeout de 30s en servicios |
| **Valores hardcodeados** | ⚠️ Sí (posiciones de prueba) | ❌ No (recibe trayectorias de MoveIt) |
| **Callback de feedback** | ⚠️ Vacío (`print(1)`) | N/A (es servidor) |
| **Nombre del nodo dinámico** | ✅ Usa `DOBOT_TYPE` env var | ✅ Usa `DOBOT_TYPE` env var |
| **Estado del código** | 🔴 Código roto y de prueba | 🟢 Producción funcional |

---

## 3. Análisis de Dependencias

### Búsqueda realizada

Se ejecutó la siguiente búsqueda en todo el workspace:

```bash
grep -rn "servo_action" src/ --include="*.py" --include="*.xml" --include="*.yaml" --include="*.launch.py"
```

### Resultados

| Archivo | Línea | Contexto |
|---|---|---|
| `servo_action/servo_action/action_move_client.py` | 3 | `#ros2 run servo_action action_move_client` (comentario interno) |
| `servo_action/setup.py` | 3 | `package_name = 'servo_action'` (definición del propio paquete) |
| `servo_action/setup.py` | 23 | `servo_action.action_move_client:main` (entry point propio) |
| `servo_action/setup.py` | 24 | `servo_action.Joint_Position:main` (entry point propio) |
| `servo_action/package.xml` | 4 | `<name>servo_action</name>` (nombre del propio paquete) |

### Conclusión de dependencias

**Cero dependencias externas.** Todas las referencias a `servo_action` provienen exclusivamente de archivos dentro del propio paquete. Ningún launch file, ningún otro paquete, ningún archivo de configuración YAML y ningún script Python externo importa o referencia `servo_action`.

---

## 4. Problemas Encontrados en `servo_action/`

### 4.1 `Joint_Position.py` — Variable indefinida (CÓDIGO ROTO)

```python
# Línea 27 de Joint_Position.py
msg2.position = joint  # ← ERROR: 'joint' nunca fue definida
```

La variable `joint` no existe en ningún ámbito del archivo. No se define como parámetro, no se recibe de ninguna suscripción, ni se inicializa en ninguna parte del código. **Este script lanza un `NameError` inmediatamente al ejecutarse** y nunca ha podido funcionar correctamente.

Además, el método `listener_callback()` contiene un `while True:` bloqueante en el constructor, lo que impediría que el nodo ROS 2 funcione correctamente con `rclpy.spin()`.

### 4.2 `action_move_client.py` — Valores hardcodeados sin seguridad

```python
# Líneas 22-28 de action_move_client.py
joint = 0.1
while True:
    if joint > 2:
        joint = 0.1
    goal_msg.trajectory.points.append(
        trajectory_msgs.msg.JointTrajectoryPoint(
            positions=[joint, 0.4, joint, joint, 0.2, 0.4]  # ← valores fijos de prueba
        ))
```

**Problemas identificados:**
- **Valores hardcodeados:** Las posiciones articulares son valores fijos de prueba, no configurables.
- **Bucle infinito sin control:** El `while True` envía comandos sin parar, sin mecanismo de detención segura.
- **Sin verificación de límites articulares:** No valida que las posiciones estén dentro de los rangos seguros del robot.
- **Callback de feedback vacío:** `feedback_callback` solo hace `print(1)`, sin lógica útil.
- **Sin manejo de errores:** No hay try/except, no se verifica si el servidor respondió correctamente.
- **Falta `rclpy.spin()` efectivo:** El bucle infinito en `send_goal()` bloquea antes de llegar a `rclpy.spin()`.
- **Typo en variable:** Usa `mane` en lugar de `name` para `os.getenv("DOBOT_TYPE")`.

### 4.3 `package.xml` — Metadatos incompletos

```xml
<description>TODO: Package description</description>
<maintainer email="xxxx@todo.todo">dobot</maintainer>
<license>TODO: License declaration</license>
```

El paquete nunca fue configurado correctamente: descripción, email de mantenedor y licencia son placeholders sin rellenar.

---

## 5. Cómo `action_move_server.py` + `joint_states.py` Cubren la Funcionalidad Completa

### 5.1 `action_move_server.py` — Servidor de trayectorias

Este archivo implementa el **servidor de acciones `FollowJointTrajectory`**, que es el componente esencial para que MoveIt 2 pueda controlar el robot Dobot CR5. Sus capacidades incluyen:

- **ActionServer completo:** Recibe trayectorias planificadas por MoveIt y las ejecuta punto a punto.
- **Conversión radianes a grados:** Convierte automáticamente las posiciones de radianes (formato MoveIt) a grados (formato Dobot).
- **Comando ServoJ:** Envía cada punto de la trayectoria al robot mediante el servicio `/dobot_bringup_ros2/srv/ServoJ`.
- **Re-habilitación del robot:** Llama a `EnableRobot` antes de cada trayectoria para resetear el modo servo.
- **MultiThreadedExecutor:** Usa 3 hilos para manejar callbacks concurrentes sin bloqueos.
- **Timeout robusto:** Espera hasta 30 segundos por el servicio `EnableRobot`, con reintentos y logging.

### 5.2 `joint_states.py` — Republicación de estados articulares

Este archivo reemplaza completamente a `Joint_Position.py` (que está roto). Implementa:

- **Suscripción a `/joint_states_robot`:** Recibe los estados articulares reales del robot Dobot.
- **Validación de datos:** Verifica que el mensaje contenga al menos 6 posiciones antes de procesarlo.
- **Republicación en `/joint_states`:** Publica los estados en el tópico estándar que MoveIt y RViz esperan.
- **Nombres correctos de articulaciones:** Usa los nombres estándar `["joint1"..."joint6"]`.

### 5.3 El cliente ya no es necesario

`servo_action/` solo proporcionaba un **cliente de prueba**. En el flujo de producción, **MoveIt 2 actúa como el cliente** de `FollowJointTrajectory`. El servidor (`action_move_server.py`) recibe las trayectorias directamente de MoveIt, eliminando la necesidad de un cliente manual separado.

---

## 6. Veredicto: ¿Es Seguro Eliminar `servo_action/`?

### ✅ SÍ — Es seguro eliminar `servo_action/` del workspace.

### Evidencia que sustenta esta decisión:

| # | Evidencia | Estado |
|---|---|---|
| 1 | **Cero dependencias externas** — `grep` confirma que ningún archivo fuera de `servo_action/` lo referencia | ✅ Verificado |
| 2 | **Código roto** — `Joint_Position.py` tiene variable `joint` indefinida (NameError fatal) | ✅ Confirmado |
| 3 | **Solo cliente de prueba** — `action_move_client.py` es un script de prueba con valores hardcodeados | ✅ Confirmado |
| 4 | **Funcionalidad completamente cubierta** — `action_move_server.py` + `joint_states.py` implementan el servidor y la republicación de joints | ✅ Verificado |
| 5 | **MoveIt reemplaza al cliente** — En producción, MoveIt actúa como cliente de FollowJointTrajectory | ✅ Verificado |
| 6 | **Metadatos incompletos** — package.xml con placeholders TODO, nunca fue finalizado | ✅ Confirmado |
| 7 | **Sin launch files** — Ningún launch file del sistema arranca nodos de `servo_action` | ✅ Verificado |

### Recomendación final

El paquete `servo_action/` es **código muerto**: un prototipo de prueba que nunca fue completado, contiene bugs fatales, y su funcionalidad ha sido completamente absorbida por `dobot_moveit`. Su eliminación **no romperá ningún componente** del sistema y **reducirá la confusión** para futuros desarrolladores que trabajen con este workspace.

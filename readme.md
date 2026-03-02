# 1) Comandos ROS2 generales (inspección y debugging)

## Ver qué hay corriendo

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> node</span><span> list</span></span>
<span class="line"><span>ros2</span><span> topic</span><span> list</span></span>
<span class="line"><span>ros2</span><span> service</span><span> list</span></span>
<span class="line"><span>ros2</span><span> action</span><span> list</span></span></code></pre></div></div></pre>

## Ver tipo de un tópico

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> info</span><span> /nombre_del_topic</span></span></code></pre></div></div></pre>

## Ver la estructura del mensaje de un tópico

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> interface</span><span> show</span><span> sensor_msgs/msg/JointState</span></span>
<span class="line"><span>ros2</span><span> interface</span><span> show</span><span> std_msgs/msg/Bool</span></span>
<span class="line"><span>ros2</span><span> interface</span><span> show</span><span> std_msgs/msg/String</span></span></code></pre></div></div></pre>

## Ver datos de un tópico (escuchar)

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> echo</span><span> /joint_states</span></span>
<span class="line"><span>ros2</span><span> topic</span><span> echo</span><span> /gripper/status</span></span></code></pre></div></div></pre>

## Ver frecuencia y ancho de banda (muy útil para cámaras)

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> hz</span><span> /joint_states</span></span>
<span class="line"><span>ros2</span><span> topic</span><span> bw</span><span> /camera/color/image_raw</span></span></code></pre></div></div></pre>

## Ver información detallada de publishers/subscribers (QoS, etc.)

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> info</span><span> /joint_states</span><span> -v</span></span></code></pre></div></div></pre>

---

# 2) Comandos para mover el brazo (por tópicos)

Tú dijiste que normalmente controlas así:

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>load_ros2</span></span>
<span class="line"><span>ros2</span><span> topic</span><span> pub</span><span> /joint_command</span><span> sensor_msgs/msg/JointState</span><span> "{name: ['joint2'], position: [-1.047]}"</span><span> --once</span></span></code></pre></div></div></pre>

Eso significa: tu pipeline está esperando comandos en **`/joint_command`** tipo `sensor_msgs/JointState`.

## 2.1 Publicar un solo joint

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>load_ros2</span></span>
<span class="line"><span>ros2</span><span> topic</span><span> pub</span><span> /joint_command</span><span> sensor_msgs/msg/JointState</span><span> \</span></span>
<span class="line"><span>"{name: ['joint4'], position: [-0.026705]}"</span><span> --once</span></span></code></pre></div></div></pre>

## 2.2 Publicar los 6 joints en un mensaje (recomendado)

Ejemplo (los que convertimos a radianes):

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>load_ros2</span></span>
<span class="line"><span>ros2</span><span> topic</span><span> pub</span><span> /joint_command</span><span> sensor_msgs/msg/JointState</span><span> \</span></span>
<span class="line"><span>"{name: ['joint1','joint2','joint3','joint4','joint5','joint6'],</span></span>
<span class="line"><span>  position: [0.549520, -0.046057, -1.457805, -0.026705, 1.607900, -0.691053]}"</span><span> --once</span></span></code></pre></div></div></pre>

## 2.3 Publicar a una tasa (cuando “--once” no basta)

Muchos controladores esperan comandos continuos:

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>load_ros2</span></span>
<span class="line"><span>ros2</span><span> topic</span><span> pub</span><span> -r</span><span> 30</span><span> /joint_command</span><span> sensor_msgs/msg/JointState</span><span> \</span></span>
<span class="line"><span>"{name: ['joint1','joint2','joint3','joint4','joint5','joint6'],</span></span>
<span class="line"><span>  position: [0.549520, -0.046057, -1.457805, -0.026705, 1.607900, -0.691053]}"</span></span></code></pre></div></div></pre>

(Cortas con `Ctrl+C`)

## 2.4 Ver el estado real del robot

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> echo</span><span> /joint_states</span></span></code></pre></div></div></pre>

> Nota crítica: ROS normalmente maneja joints en  **radianes** . Si publicas grados, se rompe.

---

# 3) Comandos para mover el brazo (por Action: FollowJointTrajectory)

Esto aplica cuando estás usando el servidor de acción (por ejemplo el del robot real en `dobot_moveit/action_move_server.py` o un controlador tipo FollowJointTrajectory).

## 3.1 Ver qué action existe realmente

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> action</span><span> list</span><span> |</span><span> grep</span><span> follow_joint_trajectory</span></span></code></pre></div></div></pre>

## 3.2 Enviar un goal con `ros2 action send_goal`

Ejemplo típico:

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> action</span><span> send_goal</span><span> \</span></span>
<span class="line"><span>/cr20_group_controller/follow_joint_trajectory </span><span>\</span></span>
<span class="line"><span>control_msgs/action/FollowJointTrajectory </span><span>\</span></span>
<span class="line"><span>"{trajectory: {joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6'],</span></span>
<span class="line"><span>               points: [{positions: [0.549520, -0.046057, -1.457805, -0.026705, 1.607900, -0.691053],</span></span>
<span class="line"><span>                         time_from_start: {sec: 2, nanosec: 0}}]}}"</span></span></code></pre></div></div></pre>

> Si tu action se llama distinto, cambia solo el nombre `/cr20_group_controller/...`.

---

# 4) Comandos del gripper (lo que ya funciona)

En tu integración actual (topic-based):

## 4.1 Ver estado del gripper

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> echo</span><span> /gripper/status</span></span></code></pre></div></div></pre>

## 4.2 Cerrar gripper

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> pub</span><span> /gripper/command</span><span> std_msgs/msg/Bool</span><span> "{data: true}"</span><span> --once</span></span></code></pre></div></div></pre>

## 4.3 Abrir gripper

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> pub</span><span> /gripper/command</span><span> std_msgs/msg/Bool</span><span> "{data: false}"</span><span> --once</span></span></code></pre></div></div></pre>

---

# 5) Servicios (la forma “profesional” que vamos a implementar)

**Qué cambia con services vs tópicos:**

* Con tópicos: publicas y “esperas” que pase.
* Con servicios: haces request y recibes response ( **confirmación** ). Esto es mejor para robot real.

## 5.1 Ejemplo con servicio estándar `std_srvs/Trigger`

(Solo como ejemplo de sintaxis):

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> service</span><span> call</span><span> /gripper/close</span><span> std_srvs/srv/Trigger</span><span> "{}"</span></span>
<span class="line"><span>ros2</span><span> service</span><span> call</span><span> /gripper/open</span><span>  std_srvs/srv/Trigger</span><span> "{}"</span></span>
<span class="line"><span>ros2</span><span> service</span><span> call</span><span> /gripper/state</span><span> std_srvs/srv/Trigger</span><span> "{}"</span></span></code></pre></div></div></pre>

## 5.2 Ver servicios disponibles

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> service</span><span> list</span><span> |</span><span> grep</span><span> gripper</span></span></code></pre></div></div></pre>

Cuando los implementemos, tu lógica pick&place (C++/MoveIt) podrá hacer:

* `call close → respuesta: success + objeto agarrado`
* si `success==false`, reintentar/bajar/abort.

---

# 6) Tiempo de simulación (muy importante)

Si Isaac publica `/clock`, en tus nodos debes usar `use_sim_time:=true`.

## Ver si hay clock

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> list</span><span> |</span><span> grep</span><span> clock</span></span>
<span class="line"><span>ros2</span><span> topic</span><span> echo</span><span> /clock</span><span> --once</span></span></code></pre></div></div></pre>

---

# 7) Cámara (cuando la integremos)

Comandos típicos para comprobar que está publicando:

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> list</span><span> |</span><span> grep</span><span> camera</span></span>
<span class="line"><span>ros2</span><span> topic</span><span> echo</span><span> /camera/color/camera_info</span><span> --once</span></span>
<span class="line"><span>ros2</span><span> topic</span><span> hz</span><span> /camera/color/image_raw</span></span></code></pre></div></div></pre>

Si usas RViz o rqt:

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>rqt_image_view</span></span></code></pre></div></div></pre>

---

# 8) “Checklist” de verificación rápida (cuando algo falla)

1. ¿Existe el tópico/action/servicio?

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> list</span><span> |</span><span> grep</span><span> joint</span></span>
<span class="line"><span>ros2</span><span> action</span><span> list</span><span> |</span><span> grep</span><span> follow</span></span>
<span class="line"><span>ros2</span><span> service</span><span> list</span><span> |</span><span> grep</span><span> gripper</span></span></code></pre></div></div></pre>

2. ¿Ves datos entrando?

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> echo</span><span> /joint_states</span><span> --once</span></span>
<span class="line"><span>ros2</span><span> topic</span><span> echo</span><span> /gripper/status</span><span> --once</span></span></code></pre></div></div></pre>

3. ¿QoS mismatch? (especialmente sensores/cámaras)

<pre><div class="not-prose my-0 flex w-full flex-col overflow-clip border border-border text-text-primary rounded-lg not-prose relative" data-code-block="true"><div class="border-border flex items-center justify-between border-b px-4 py-2"><div class="flex items-center gap-2"><svg width="14" stroke-width="1.5" height="14" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg" color="currentColor" class="text-text-secondary"></svg><span class="text-text-secondary text-sm font-medium">Bash</span></div></div><div class="code-block_container__lbMX4"><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> topic</span><span> info</span><span> /camera/color/image_raw</span><span> -v</span></span></code></pre></div></div></pre>

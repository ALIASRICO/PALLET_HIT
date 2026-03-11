# EJECUCION DESPALETIZADOR 

---

## SIEMPRE

```
source /opt/ros/jazzy/setup.bash
source ~/dobot_ws/install/setup.bash
```

### Terminal 1

<pre><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>export</span><span> IP_address</span><span>=</span><span>192.168.5.1</span></span>
<span class="line"><span>export</span><span> DOBOT_TYPE</span><span>=</span><span>cr20</span></span>
<span class="line"></span>
<span class="line"><span>ros2</span><span> launch</span><span> dobot_bringup_v4</span><span> dobot_bringup_ros2.launch.py</span></span></code></pre></pre>

### Terminal 2

<pre><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>export</span><span> DOBOT_TYPE</span><span>=</span><span>cr20</span></span>
<span class="line"></span>
<span class="line"><span>ros2</span><span> launch</span><span> cr20_moveit</span><span> real_robot.launch.py</span></span></code></pre></pre>

### Terminal 3

<pre><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>~</span><span>/dobot_ws/scripts/run_yolo.sh</span></span></code></pre></pre>

### Terminal 4

```export
export IP_address=192.168.5.1
ros2 run dobot_moveit action_move_server
```

### Terminal 5

<pre><pre class="shiki github-dark shiki-code-block" tabindex="0"><code class="whitespace-pre-wrap break-words"><span class="line"><span>ros2</span><span> run</span><span> dobot_moveit</span><span> depalletizer</span></span></code></pre></pre>

# README — Proyecto Final Brazo Robótico AgileX Nero (ROS 2 Jazzy + MuJoCo)

Guía técnica y manual de traspaso para el grupo sobre la implementación de la **Entrega Final** en el repositorio oficial [`Mikson16/iPre_nero`](https://github.com/Mikson16/iPre_nero), tomando como base el trabajo previo de la **Parte 1 (`codigo_oficial_grupo`)**.

---

## 1. Contexto Inicial

### Relación con la Parte 1 (`codigo_oficial_grupo`)
En la Parte 1 del proyecto el grupo desarrolló y validó en MuJoCo un controlador funcional de *Pick & Lift* sobre cinta transportadora (`codigo_oficial_grupo/`):
- `mujoco_viewer.py`: puente ROS 2 $\leftrightarrow$ MuJoCo que publica `/feedback/joint_states` (200 Hz) y ejecuta `/control/move_j` y `/control/joint_states`.
- `tarea_Nero_new.py`: controlador en $SE(3)$ sobre `pinch_site` usando logaritmo matricial (`scipy.linalg.logm`), Jacobiano del cuerpo $J_b$, estimador de velocidad en Y y máquina de estados (`INICIO -> BAJAR -> AGARRAR -> SUBIR`).
- `scene_cube.xml`, `nero.xml`, `cinta.xml` y `assets/`: geometría exacta de la cinta (`X = -0.28 m`, superficie en `Z = 0.064 m`), el brazo Nero y el cubo de 4 cm (`0.05 kg`, fricción `2.5`, `condim="4"`).

### Qué se tomó como base y qué se mejoró para la Entrega Final (`iPre_nero`)
Todo el desarrollo de la Entrega Final se integró dentro del paquete `nero_ctrl` de `~/iPre_nero`, manteniendo intactos los archivos base del repositorio (`IK_node.py`, `main_node.py`, `can_module.py`, `calibration_module.py`, `agx_arm_ros`) y empaquetando/mejorando el trabajo del grupo:
1. **Controlador $SE(3)$ compartido (`comun/cinematica_se3.py`):** se extrajo la matemática de `tarea_Nero_new.py` a una librería reutilizable con **Mínimos Cuadrados Amortiguados (DLS)**, sesgo en el espacio nulo (`Q_NOMINAL_NERO`), orientación `R_PINZA_ABAJO_Y` (error $< 0.8\text{ mm}$ en toda la cinta y mesa) y **resolución IK convergente desde el último comando suave `theta_cmd`** (eliminando el lazo integrador sobre `q_current` atrasado que causaba rebotes/oscilaciones).
2. **Compensación de gravedad realista en simulación (`nero.xml`):** se activó `gravcomp="1"` **solo en los eslabones del brazo** (`link1..7`, `gripper_link1..2`) para replicar el firmware real del Nero, manteniendo 100% gravedad terrestre (`-9.81 m/s²`) en los cubos.
3. **Puente MuJoCo empaquetado (`comun/simulador_mujoco_bridge.py`):** evolución de `mujoco_viewer.py` con carga automática de escenas (`--scene-name`), velocidad inicial de cinta (`--init-ctrl`) y reciclaje continuo de cubos (`--loop-cubes`) para probar evasión de obstáculos.
4. **Compatibilidad directa Simulación $\leftrightarrow$ Brazo Real:** todos los `launch` soportan `mode:=sim` y `mode:=real`, y todos los nodos escuchan los mismos tópicos oficiales (`/feedback/joint_states`, `/control/move_j`) además de los tópicos del sensor real (`/sensor/cube_distance`, `/cube/position`, `/cubes/positions`).

### Instalación rápida para cualquier integrante del grupo
Partiendo de Ubuntu 24.04 (nativo o WSL2) con ROS 2 Jazzy y `~/iPre_nero` clonado:
```bash
# 1. Copiar los archivos modificados/creados sobre ~/iPre_nero/src/
cp -r "<ruta>/Entrega Final/archivos_modificados_iPre_nero/src/"* ~/iPre_nero/src/

# 2. Compilar el paquete nero_ctrl
source /opt/ros/jazzy/setup.bash
cd ~/iPre_nero
colcon build --symlink-install --packages-select nero_ctrl
source ~/iPre_nero/install/setup.bash
```

---

## 2. Estructura Base

### Organización de archivos en `~/iPre_nero/src/nero_ctrl/`
```text
nero_ctrl/
├── launch/
│   ├── nero_can_ctrl.launch.py               # (original iPre_nero)
│   ├── nero_calibration.launch.py            # (original iPre_nero)
│   ├── tarea1_1_gemelo_digital.launch.py     # Tarea 1.1: visor pasivo MuJoCo (mode:=demo | real)
│   ├── tarea1_2_recoger_cinta.launch.py      # Tarea 1.2: pick en cinta (mode:=sim | real)
│   ├── tarea1_3_piramide.launch.py           # Tarea 1.3: pirámide 3-2-1 (mode:=sim | real)
│   ├── tarea2_1_evasion_estatica.launch.py   # Tarea 2.1: evasión fija (mode:=sim | real)
│   └── tarea2_2_evasion_movil.launch.py      # Tarea 2.2: evasión móvil vel. cte. (mode:=sim | real)
├── models/escena_cinta_grupo/
│   ├── assets/                               # Meshes STL/OBJ del brazo y gripper
│   ├── nero.xml                              # Brazo Nero (gravcomp="1" solo en eslabones)
│   ├── cinta.xml                             # Cinta transportadora (superficie Z = 0.064 m)
│   ├── scene_cube.xml                        # Escena Tareas 1.1 y 1.2 (1 cubo en cinta)
│   ├── scene_piramide.xml                    # Escena Tarea 1.3 (mesa + 6 cubos libres 6 GDL)
│   └── scene_evasion_cinta.xml               # Escena Tareas 2.1 y 2.2 (cinta + 2 cubos en bucle)
└── nero_ctrl/
    ├── controller_node.py                    # Controlador reactivo puro de evasión (Tareas 2.1 y 2.2)
    ├── comun/
    │   ├── cinematica_se3.py                 # Librería IK SE(3) convergente (DLS + espacio nulo)
    │   └── simulador_mujoco_bridge.py        # Ejecutable `nero_sim_bridge`
    ├── tarea1_manejo/
    │   ├── gemelo_digital_mujoco.py          # Ejecutable `tarea1_1_gemelo`
    │   ├── emulador_estado_real.py           # Ejecutable `tarea1_1_emulador`
    │   ├── recoger_cubo_cinta.py             # Ejecutable `tarea1_2_recoger_cinta`
    │   └── armar_piramide.py                 # Ejecutable `tarea1_3_piramide`
    └── tarea2_control/
        ├── tarea2_1_estatica.py              # Ejecutable `tarea2_1_estatica`
        └── tarea2_2_movil.py                 # Ejecutable `tarea2_2_movil`
```

### Nodos activos según el tipo de tarea

| Categoría | Nodos que actúan | Rol |
|---|---|---|
| **Backend (Simulación)** | `nero_sim_bridge` | Simula MuJoCo, publica `/feedback/joint_states` (200 Hz) y ejecuta `/control/move_j`. |
| **Backend (Real)** | `agx_arm_ctrl_single_node` | Comunica con el brazo físico vía CAN (`can0` + `pyAgxArm`), publica `/feedback/joint_states` y ejecuta `/control/move_j`. |
| **Sensor Real (por crear)** | `arduino_sensor_node` *(puente `pyserial`)* | Lee el sensor de distancia del Arduino y publica `std_msgs/Float64` en `/sensor/cube_distance`. |
| **Tarea 1.1** | `tarea1_1_gemelo` (+ `tarea1_1_emulador` en demo) | Visor MuJoCo pasivo (solo lectura) que refleja el estado del brazo y del cubo. |
| **Tarea 1.2** | `tarea1_2_recoger_cinta` | Estimador de velocidad + máquina de estados *pick & lift* en cinta. |
| **Tarea 1.3** | `tarea1_3_piramide` | Planificador secuencial de 6 ciclos *pick & place* para pirámide 3-2-1. |
| **Tareas 2.1 y 2.2** | `controller_node` + (`tarea2_1_estatica` ó `tarea2_2_movil`) | El nodo de tarea publica referencia nominal en `/control/nominal_target`; `controller_node` aplica la ley reactiva de evasión y publica en `/control/move_j`. |

### Arquitectura en Caso Simulación (estado actual funcionando)
```text
┌────────────────────────────────────────────────────────────────────────────┐
│                           CASO SIMULACIÓN (MuJoCo)                         │
│                                                                            │
│  [nero_sim_bridge]  ── /feedback/joint_states (joint1..7, gripper, cubos) ─┼──▶ [Nodos Tarea 1.x]
│         ▲                                                                  │            │
│         │               ┌── /control/nominal_target ◀── [tarea2_1 / 2_2]   │            │
│         │               ▼                                                  │            │
│         └──── /control/move_j ◀──── [controller_node (Parte 2)] ◀──────────┘            │
│         └──── /control/move_j ◀─────────────────────────────────────────────────────────┘
└────────────────────────────────────────────────────────────────────────────┘
```

### Arquitectura en Caso Real (Laboratorio: CAN + Brazo Nero + Arduino)
```text
┌────────────────────────────────────────────────────────────────────────────┐
│                       CASO REAL (Laboratorio + Hardware)                   │
│                                                                            │
│  [Brazo AgileX Nero] ◀══ USB-CAN (can0, 1 Mbps) ══▶ [agx_arm_ctrl]         │
│                                                           │   ▲            │
│               /feedback/joint_states (encoders reales) ───┘   │            │
│                                        │          /control/move_j          │
│                                        ▼                      │            │
│  [Arduino + Sensor] ── USB Serial ──▶ [nodo_puente_arduino]   │            │
│                                        │                      │            │
│                         /sensor/cube_distance (Float64)       │            │
│                                        ▼                      │            │
│             ┌─────────────────────────────────────────┐       │            │
│             │ • Tarea 1.1: gemelo_digital_mujoco      │ (solo lectura)     │
│             │ • Tarea 1.2: tarea1_2_recoger_cinta  ───┼───────┘            │
│             │ • Tarea 1.3: tarea1_3_piramide       ───┼───────┘            │
│             │ • Parte 2:   tarea2_x -> controller_node┼───────┘            │
│             └─────────────────────────────────────────┘                    │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Apartados de Cada Tarea (Ejecución y Flujo)

*(En todas las terminales de Ubuntu asegúrate de haber cargado el entorno: `source /opt/ros/jazzy/setup.bash && source ~/iPre_nero/install/setup.bash`, o usar el alias `nero_final` si usas el `~/.bashrc` configurado).*

### Referencia — Parte 1 (`codigo_oficial_grupo`)
- **Ejecución:**
  - Terminal 1: `cd "<ruta>/Parte 1/codigo_oficial_grupo" && python3 mujoco_viewer.py scene_cube.xml`
  - Terminal 2: `cd "<ruta>/Parte 1/codigo_oficial_grupo" && NERO_SCENE_XML=./scene_cube.xml python3 tarea_Nero_new.py`
  - Terminal 3 *(opcional, velocidad cinta)*: `ros2 topic pub --once /control/joint_states sensor_msgs/msg/JointState "{name: [cube_motor], position: [0.03]}"`
- **Flujo:** `mujoco_bridge` publica `/feedback/joint_states`; `tarea_Nero_new.py` estima $v_y$, calcula IK en $SE(3)$ y ejecuta `INICIO -> BAJAR -> AGARRAR -> SUBIR`.

---

### Tarea 1.1 — Gemelo Digital en MuJoCo *(10 pts)*
- **Ejecución (prueba en casa sin hardware):**
  ```bash
  ros2 launch nero_ctrl tarea1_1_gemelo_digital.launch.py mode:=demo
  ```
- **Ejecución (laboratorio con brazo real + Arduino):**
  ```bash
  # (Con can0 activo, agx_arm_ctrl corriendo y nodo Arduino publicando en /sensor/cube_distance)
  ros2 launch nero_ctrl tarea1_1_gemelo_digital.launch.py mode:=real distance_unit:=cm sensor_y_origin:=-0.25 sensor_direction:=1.0
  ```
- **Flujo del código:** `gemelo_digital_mujoco.py` es estrictamente **pasivo (solo lectura)**: se suscribe a `/feedback/joint_states`, `/sensor/cube_distance` y `/cube/position`, asigna `joint1..7`, los dos dedos simétricos (`±gripper/2`) y el desplazamiento del cubo a `data.qpos`, ejecuta solo cinemática directa (`mj_forward`) y actualiza la ventana a 60 Hz sin correr `mj_step` ni publicar comandos.

---

### Tarea 1.2 — Recoger Cubo en Cinta Transportadora (>80% éxito) *(20 pts)*
- **Ejecución (simulación en una terminal):**
  ```bash
  ros2 launch nero_ctrl tarea1_2_recoger_cinta.launch.py mode:=sim cube_speed:=0.03
  ```
- **Ejecución (laboratorio, probar primero elevado con `z_offset_seguridad:=0.10`):**
  ```bash
  ros2 launch nero_ctrl tarea1_2_recoger_cinta.launch.py mode:=real can_port:=can0 distance_unit:=cm z_offset_seguridad:=0.10
  ```
- **Flujo del código:** `recoger_cubo_cinta.py` lee la posición del cubo (sensor 1D, punto 3D o fallback MuJoCo), filtra su velocidad $v_y$ con rechazo de picos y ejecuta la máquina de estados `ESPERA -> ACERCAMIENTO (predicción con lead_time=0.22 s) -> BAJAR -> AGARRAR -> SUBIR (-> DEPOSITAR si depositar_y_repetir:=true)`, verificando el agarre y reportando la tasa de éxito acumulada (`exitos/intentos`).

---

### Tarea 1.3 — Armado de Pirámide 3-2-1 *(30 pts)*
- **Ejecución (simulación en una terminal):**
  ```bash
  ros2 launch nero_ctrl tarea1_3_piramide.launch.py mode:=sim
  ```
- **Ejecución (laboratorio, probar primero elevado con `z_offset_seguridad:=0.10`):**
  ```bash
  ros2 launch nero_ctrl tarea1_3_piramide.launch.py mode:=real can_port:=can0 z_mesa:=0.064 z_offset_seguridad:=0.10
  ```
- **Flujo del código:** `armar_piramide.py` ejecuta 6 ciclos secuenciales de *pick & place* sobre 6 cubos libres en `scene_piramide.xml` (`3` en base a `Z = 0.084 m`, `2` en piso medio a `Z = 0.124 m`, `1` en punta a `Z = 0.164 m`). Usa trayectorias *minimum-jerk* (`smootherstep`), posado suave (`+2 mm`), retracción vertical y fila de la pirámide alineada en `X` (separación `4.4 cm`) perpendicular a la apertura de dedos en `Y` (`R_PINZA_ABAJO_Y`) para evitar colisiones con cubos vecinos.

---

### Tarea 2.1 — Evasión Estática sobre la Cinta *(30 pts)*
- **Ejecución (simulación en una terminal):**
  ```bash
  ros2 launch nero_ctrl tarea2_1_evasion_estatica.launch.py mode:=sim cube_speed:=0.06
  ```
- **Ejecución (laboratorio):**
  ```bash
  ros2 launch nero_ctrl tarea2_1_evasion_estatica.launch.py mode:=real can_port:=can0 distance_unit:=cm z_offset_seguridad:=0.08
  ```
- **Flujo del código (arquitectura de 3 nodos):**
  1. `nero_sim_bridge` (o `agx_arm_ctrl`) publica `/feedback/joint_states`.
  2. `tarea2_1_estatica.py` publica a 40 Hz la referencia nominal fija `(-0.28, 0.0, 0.094 m, vy=0)` en `/control/nominal_target`.
  3. `controller_node.py` (controlador reactivo puro) proyecta los cubos en `t_anticipacion = 0.32 s`, calcula la activación suave *smoothstep* entre `radio_alerta = 0.085 m` y `radio_pleno = 0.065 m`, aplica filtro asimétrico (`tau_subida = 0.04 s`, `tau_bajada = 0.22 s`) elevando `delta_z_evasion = +0.05 m` (`Z = 0.144 m`) y envía el comando $SE(3)$ a `/control/move_j`.

---

### Tarea 2.2 — Evasión Móvil a Velocidad Constante sobre la Cinta *(10 pts)*
- **Ejecución (simulación en una terminal):**
  ```bash
  ros2 launch nero_ctrl tarea2_2_evasion_movil.launch.py mode:=sim cube_speed:=0.06
  # Opcional: ajustar velocidad o rango del brazo:
  # ros2 launch nero_ctrl tarea2_2_evasion_movil.launch.py mode:=sim cube_speed:=0.06 velocidad_y:=0.10 y_min:=-0.20 y_max:=0.20
  ```
- **Ejecución (laboratorio):**
  ```bash
  ros2 launch nero_ctrl tarea2_2_evasion_movil.launch.py mode:=real can_port:=can0 distance_unit:=cm z_offset_seguridad:=0.08
  ```
- **Flujo del código (arquitectura de 3 nodos):**
  1. `tarea2_2_movil.py` genera una trayectoria trapezoidal ida y vuelta a **velocidad constante** (`0.10 m/s`) entre `y_min = -0.20 m` y `y_max = +0.20 m` (con frenado breve solo en los extremos) y publica `[x, y, z, 0, vy, 0, 0.01]` en `/control/nominal_target`.
  2. `controller_node.py` (el mismo nodo sin cambios) usa la **velocidad relativa** $v_{\text{rel}} = v_{y,\text{cubo}} - v_{y,\text{brazo}}$ para anticipar tanto cruces frontales como sobrepasos y elevar el efector `+0.05 m` sin detener el patrullaje lineal.

---

## 4. Implementación Real en el Laboratorio (Qué falta agregar y configurar)

### A. Configuración del bus CAN y del brazo físico AgileX Nero
1. **Requisitos instalados en Ubuntu 24.04:** `can-utils`, `ethtool`, `iproute2`, `pyAgxArm`, `python-can`, `pyserial` (ya cubiertos por `scripts_setup/01_instalar_ros2_y_deps.sh`).
2. **Solo si usas Windows + WSL2:** instala `usbipd-win` en PowerShell (Administrador) y adjunta el adaptador USB-CAN oficial del brazo y el Arduino:
   ```powershell
   winget install --exact dorssel.usbipd-win   # solo la primera vez
   usbipd list
   usbipd bind --busid <BUSID_CAN>             # solo la primera vez
   usbipd attach --wsl --busid <BUSID_CAN>
   usbipd attach --wsl --busid <BUSID_ARDUINO>
   ```
3. **Activar la interfaz `can0` a `1 Mbit/s` en Ubuntu:**
   ```bash
   sudo modprobe can can_raw can_dev gs_usb    # necesario en WSL2
   bash ~/iPre_nero/src/agx_arm_ros/scripts/can_activate.sh can0 1000000
   # Verificar que hay tráfico del brazo (Ctrl+C para salir):
   candump -tz can0
   ```
4. **Habilitar / deshabilitar motores (con área despejada y E-stop a mano):**
   ```bash
   ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
   # Al terminar o ante cualquier imprevisto:
   ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
   ```
5. *(Solo si el brazo perdió su cero mecánico, firmware $\ge$ V111, sin otros nodos usando `can0`):*
   ```bash
   ros2 launch nero_ctrl nero_calibration.launch.py channel:=can0 bitrate:=1000000 firmware:=v111 joint_index:=1
   # En otra terminal tras llevar la junta a su cero mecánico:
   ros2 service call /calibrate_joint std_srvs/srv/Trigger "{}"
   ```

### B. Parámetros a calibrar para que coincidan con el laboratorio

| Parámetro | Archivo / Launch | Valor actual (sim) | Qué medir y ajustar en el laboratorio |
|---|---|---|---|
| `x_cinta` / `cube_x` | `tarea1_2`, `tarea2_1`, `tarea2_2`, `controller_node` | `-0.28 m` | Distancia en X desde `base_link` al centro longitudinal de la cinta real. |
| `z_mesa` / `cube_z` / `z_bajo` | `tarea1_2`, `tarea1_3`, `tarea2_1`, `tarea2_2` | `0.064` / `0.084` / `0.094 m` | Altura real de la superficie de la banda/mesa respecto a la base del robot (`cube_z = z_banda + 0.02 m`). |
| `sensor_y_origin` | `tarea1_1`, `tarea1_2`, `controller_node` | `-0.25 m` | Coordenada Y (marco `base_link`) donde el sensor de distancia marca `0`. |
| `sensor_direction` | `tarea1_1`, `tarea1_2`, `controller_node` | `+1.0` | `+1.0` si al aumentar la distancia medida el cubo avanza hacia `+Y`; `-1.0` si avanza hacia `-Y`. |
| `distance_unit` | Todos los launch | `"m"` | Cambiar a `"cm"` si el Arduino imprime la distancia en centímetros. |
| `z_offset_seguridad` | Todos los launch | `0.0 m` | Empezar siempre en `0.10` (10 cm arriba) para validar trayectorias en el aire antes de bajar a `0.0`. |
| `delta_z_evasion`, `radio_alerta`, `radio_pleno` | `controller_node.py` | `0.05` / `0.085` / `0.065 m` | Ajustar según velocidad real de la cinta y despeje deseado sobre el cubo. |
| `plan_cubos` (pick/place) | `armar_piramide.py` | `Y = ±0.16` $\rightarrow$ `Y = 0.0` | Ajustar las 6 coordenadas de acopio y el centro (`piramide_x`, `piramide_y`) en la mesa física. |

### C. Nodos que faltan crear para conectar el Arduino y sensores reales

No hace falta modificar ninguno de los nodos ya desarrollados; solo falta agregar:

1. **Nodo puente `arduino_sensor_node` (obligatorio para Tareas 1.1, 1.2 y 2.1/2.2 con 1 cubo):**
   - El Arduino debe enviar una lectura numérica por línea vía Serial (ej. `Serial.println(dist_cm);` a 20–30 Hz).
   - En Ubuntu, dar permiso al puerto (`sudo chmod 666 /dev/ttyACM0`) y correr un nodo Python con `pyserial` que publique `std_msgs/msg/Float64` en **`/sensor/cube_distance`**:
     ```python
     # Esquema mínimo del nodo puente (pyserial -> /sensor/cube_distance)
     import serial, rclpy
     from rclpy.node import Node
     from std_msgs.msg import Float64

     class ArduinoBridge(Node):
         def __init__(self):
             super().__init__("arduino_sensor_node")
             self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.05)
             self.pub = self.create_publisher(Float64, "/sensor/cube_distance", 10)
             self.create_timer(0.02, self.read_serial)

         def read_serial(self):
             line = self.ser.readline().decode("utf-8", errors="ignore").strip()
             try:
                 val = float(line)
                 self.pub.publish(Float64(data=val))
             except ValueError:
                 pass
     ```
   - Apenas `/sensor/cube_distance` empiece a publicar, `gemelo_digital_mujoco`, `recoger_cubo_cinta` y `controller_node` tomarán automáticamente esa medición.

2. **Nodo detector multi-cubo *(solo si en 2.1/2.2 colocan varios cubos simultáneos en la cinta)*:**
   - Un sensor de distancia 1D solo ve el cubo más cercano a él. Si en el laboratorio necesitan detectar varios cubos a la vez (por ejemplo con cámara RealSense o múltiples sensores), basta publicar sus coordenadas `(x, y, z)` en el marco `base_link` como `geometry_msgs/msg/PoseArray` en el tópico **`/cubes/positions`**, al cual `controller_node.py` ya está suscrito.

### D. Secuencia segura recomendada el primer día en el laboratorio
1. Activar CAN (`can0`) y verificar tramas con `candump -tz can0`.
2. Lanzar **Tarea 1.1** (`mode:=real`) con los motores deshabilitados, mover el brazo a mano y deslizar un cubo frente al sensor para comprobar visualmente en MuJoCo que los signos y escalas coinciden.
3. Probar **1.2**, **1.3**, **2.1** y **2.2** primero con `z_offset_seguridad:=0.10` antes de habilitar el contacto con la cinta o los cubos reales (`z_offset_seguridad:=0.0`).

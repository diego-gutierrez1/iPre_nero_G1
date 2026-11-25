#!/usr/bin/env python3
"""
gemelo_digital_mujoco.py — Tarea 1.1 (Entrega Final)
====================================================
Visor MuJoCo pasivo (gemelo digital) que refleja en tiempo real:
  1. El estado articular del brazo real AgileX Nero + gripper (/feedback/joint_states).
  2. La posición del cubo real sobre la cinta transportadora, aceptando:
     - Sensor de distancia 1D (Arduino): /sensor/cube_distance (std_msgs/Float64)
     - Posición 3D en base_link:         /cube/position        (geometry_msgs/PointStamped)
     - Fallback desde JointState:        cube_joint / cube_x_joint / cube_z_joint

IMPORTANTE (Seguridad y compatibilidad con hardware real):
  Este nodo es estrictamente SOLO LECTURA (espejo cinemático con mj_forward):
  NO avanza física (mj_step), NO publica comandos en /control/move_j y
  NO publica en /feedback/joint_states. Por eso puede correr en paralelo con
  agx_arm_ctrl (mode:=real) sin interferir con el brazo físico.
"""

import os
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]

# Origen nominal del cuerpo moving_cube en scene_cube.xml (marco mundo / base_link)
CUBE_XML_ORIGIN_X = -0.28
CUBE_XML_ORIGIN_Y = -0.25
CUBE_XML_ORIGIN_Z = 0.084


def resolve_scene_path(param_path: str = "") -> str:
    """Localiza scene_cube.xml (parámetro explícito, share de nero_ctrl o árbol fuente)."""
    if param_path:
        p = Path(os.path.expanduser(param_path)).resolve()
        if p.is_file():
            return str(p)
        raise FileNotFoundError(f"No se encontró scene_xml en: {p}")

    env_path = os.environ.get("NERO_SCENE_XML", "")
    if env_path:
        p = Path(os.path.expanduser(env_path)).resolve()
        if p.is_file():
            return str(p)

    try:
        share_dir = Path(get_package_share_directory("nero_ctrl"))
        cand = share_dir / "models" / "escena_cinta_grupo" / "scene_cube.xml"
        if cand.is_file():
            return str(cand)
    except PackageNotFoundError:
        pass

    for parent in Path(__file__).resolve().parents:
        for rel in (
            Path("models/escena_cinta_grupo/scene_cube.xml"),
            Path("src/nero_ctrl/models/escena_cinta_grupo/scene_cube.xml"),
        ):
            cand = parent / rel
            if cand.is_file():
                return str(cand)

    raise FileNotFoundError(
        "No se pudo localizar scene_cube.xml. Pasa el parámetro scene_xml:=<ruta>."
    )


class GemeloDigitalMujocoNode(Node):
    """Nodo ROS 2 pasivo que sincroniza el estado real del brazo y cubo con MuJoCo."""

    def __init__(self):
        super().__init__("gemelo_digital_mujoco")

        # Parámetros configurables desde launch o línea de comandos
        self.declare_parameter("scene_xml", "")
        self.declare_parameter("joint_state_topic", "/feedback/joint_states")
        self.declare_parameter("cube_distance_topic", "/sensor/cube_distance")
        self.declare_parameter("cube_point_topic", "/cube/position")
        self.declare_parameter("distance_unit", "m")          # 'm' o 'cm'
        self.declare_parameter("sensor_y_origin", -0.25)      # Y inicial del sensor (m)
        self.declare_parameter("sensor_direction", 1.0)       # +1.0 hacia +Y, -1.0 hacia -Y
        self.declare_parameter("cube_x_default", CUBE_XML_ORIGIN_X)
        self.declare_parameter("cube_z_default", CUBE_XML_ORIGIN_Z)
        self.declare_parameter("render_hz", 60.0)

        scene_param = str(self.get_parameter("scene_xml").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.cube_distance_topic = str(self.get_parameter("cube_distance_topic").value)
        self.cube_point_topic = str(self.get_parameter("cube_point_topic").value)
        self.distance_unit = str(self.get_parameter("distance_unit").value).strip().lower()
        self.sensor_y_origin = float(self.get_parameter("sensor_y_origin").value)
        self.sensor_direction = float(self.get_parameter("sensor_direction").value)
        self.cube_x_default = float(self.get_parameter("cube_x_default").value)
        self.cube_z_default = float(self.get_parameter("cube_z_default").value)
        self.render_hz = float(self.get_parameter("render_hz").value)

        scene_path = resolve_scene_path(scene_param)
        self.get_logger().info(f"Cargando escena MuJoCo (modo gemelo pasivo): {scene_path}")

        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)

        # Mapa de juntas del modelo MuJoCo -> dirección en data.qpos
        self._qpos_adr = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self._qpos_adr[name] = int(self.model.jnt_qposadr[i])

        missing = [j for j in ARM_JOINTS if j not in self._qpos_adr]
        if missing:
            raise RuntimeError(f"Faltan juntas del brazo en el XML: {missing}")

        # Rango del dedo del gripper (por defecto 0.0 .. 0.05 m por dedo)
        self._finger_max = 0.05
        if "gripper_joint1" in self._qpos_adr:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_joint1")
            if jid >= 0 and self.model.jnt_limited[jid]:
                self._finger_max = float(self.model.jnt_range[jid, 1])

        # Estado compartido protegido por lock entre hilo ROS 2 e hilo del viewer MuJoCo
        self._lock = threading.Lock()
        self._arm_q = np.zeros(7, dtype=float)
        self._gripper_width = 0.04
        self._cube_world_xyz = np.array(
            [self.cube_x_default, self.sensor_y_origin, self.cube_z_default],
            dtype=float,
        )
        self._arm_rx_count = 0
        self._cube_source = "inicial (sin datos)"
        self._has_dedicated_cube = False

        # QoS VOLATILE compatible con agx_arm_ctrl y mujoco_viewer
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(
            JointState, self.joint_state_topic, self._on_joint_states, qos
        )
        self.create_subscription(
            Float64, self.cube_distance_topic, self._on_cube_distance, qos
        )
        self.create_subscription(
            PointStamped, self.cube_point_topic, self._on_cube_point, qos
        )

        # Log periódico de estado cada 2 segundos
        self.create_timer(2.0, self._log_status)

        self.get_logger().info(
            "Gemelo digital listo. Suscrito a:\n"
            f"  - Brazo real:       {self.joint_state_topic} (sensor_msgs/JointState)\n"
            f"  - Sensor distancia: {self.cube_distance_topic} (std_msgs/Float64, unidad={self.distance_unit})\n"
            f"  - Posicion 3D cubo: {self.cube_point_topic} (geometry_msgs/PointStamped)"
        )

    # ------------------------------------------------------------------ Callbacks ROS 2
    def _on_joint_states(self, msg: JointState):
        pos_map = dict(zip(msg.name, msg.position))
        with self._lock:
            updated_arm = False
            for idx, jname in enumerate(ARM_JOINTS):
                if jname in pos_map:
                    self._arm_q[idx] = float(pos_map[jname])
                    updated_arm = True
            if updated_arm:
                self._arm_rx_count += 1

            # El brazo real publica 'gripper' como ancho total en metros (0.0 .. 0.10)
            if "gripper" in pos_map:
                self._gripper_width = float(pos_map["gripper"])
            elif "gripper_joint1" in pos_map:
                self._gripper_width = 2.0 * float(pos_map["gripper_joint1"])

            # Fallback: si el cubo viene en /feedback/joint_states (ej. desde simulador)
            # y no se han recibido mensajes en /sensor/cube_distance ni /cube/position
            if not self._has_dedicated_cube and "cube_joint" in pos_map:
                dx = float(pos_map.get("cube_x_joint", 0.0))
                dy = float(pos_map["cube_joint"])
                dz = float(pos_map.get("cube_z_joint", 0.0))
                self._cube_world_xyz[:] = [
                    CUBE_XML_ORIGIN_X + dx,
                    CUBE_XML_ORIGIN_Y + dy,
                    CUBE_XML_ORIGIN_Z + dz,
                ]
                self._cube_source = "feedback/joint_states (fallback)"

    def _on_cube_distance(self, msg: Float64):
        dist = float(msg.data)
        if self.distance_unit == "cm":
            dist *= 0.01
        y_world = self.sensor_y_origin + self.sensor_direction * dist
        with self._lock:
            self._cube_world_xyz[0] = self.cube_x_default
            self._cube_world_xyz[1] = y_world
            self._cube_world_xyz[2] = self.cube_z_default
            self._has_dedicated_cube = True
            self._cube_source = f"{self.cube_distance_topic} ({dist:.3f} m)"

    def _on_cube_point(self, msg: PointStamped):
        with self._lock:
            self._cube_world_xyz[0] = float(msg.point.x)
            self._cube_world_xyz[1] = float(msg.point.y)
            self._cube_world_xyz[2] = float(msg.point.z)
            self._has_dedicated_cube = True
            self._cube_source = self.cube_point_topic

    def _log_status(self):
        with self._lock:
            rx = self._arm_rx_count
            gw = self._gripper_width
            cx, cy, cz = self._cube_world_xyz.tolist()
            src = self._cube_source
        self.get_logger().info(
            f"[Gemelo] msgs_brazo={rx} | gripper={gw:.3f} m | "
            f"cubo=({cx:+.3f}, {cy:+.3f}, {cz:+.3f}) m [fuente: {src}]"
        )

    # ------------------------------------------------------- Sincronización con MuJoCo
    def apply_state_to_mujoco(self):
        """Copia el último estado real recibido a data.qpos y actualiza cinemática."""
        with self._lock:
            arm_q = self._arm_q.copy()
            gripper_width = self._gripper_width
            cube_xyz = self._cube_world_xyz.copy()

        # 1) Articulaciones del brazo
        for idx, jname in enumerate(ARM_JOINTS):
            self.data.qpos[self._qpos_adr[jname]] = arm_q[idx]

        # 2) Dedos simétricos del gripper (como no corremos mj_step, asignamos ambos)
        half = float(np.clip(gripper_width / 2.0, 0.0, self._finger_max))
        if "gripper_joint1" in self._qpos_adr:
            self.data.qpos[self._qpos_adr["gripper_joint1"]] = half
        if "gripper_joint2" in self._qpos_adr:
            self.data.qpos[self._qpos_adr["gripper_joint2"]] = -half

        # 3) Posición del cubo respecto al origen de moving_cube en scene_cube.xml
        if "cube_x_joint" in self._qpos_adr:
            self.data.qpos[self._qpos_adr["cube_x_joint"]] = cube_xyz[0] - CUBE_XML_ORIGIN_X
        if "cube_joint" in self._qpos_adr:
            self.data.qpos[self._qpos_adr["cube_joint"]] = cube_xyz[1] - CUBE_XML_ORIGIN_Y
        if "cube_z_joint" in self._qpos_adr:
            self.data.qpos[self._qpos_adr["cube_z_joint"]] = cube_xyz[2] - CUBE_XML_ORIGIN_Z

        # Solo cinemática directa (sin dinámica/colisiones)
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)


def main(args=None):
    rclpy.init(args=args if args is not None else sys.argv)
    node = GemeloDigitalMujocoNode()

    # ROS 2 spin en hilo secundario; MuJoCo viewer corre en el hilo principal
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    period = 1.0 / max(node.render_hz, 10.0)
    try:
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            while viewer.is_running() and rclpy.ok():
                t0 = time.perf_counter()
                node.apply_state_to_mujoco()
                viewer.sync()
                dt = period - (time.perf_counter() - t0)
                if dt > 0.0:
                    time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

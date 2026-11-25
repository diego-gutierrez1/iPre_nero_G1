#!/usr/bin/env python3
"""
recoger_cubo_cinta.py — Tarea 1.2 (Entrega Final)
=================================================
Controlador robusto en SE(3) para interceptar, agarrar y levantar un cubo de 4 cm
en movimiento sobre la cinta transportadora (objetivo: tasa de éxito > 80%).

Evolución de `codigo_oficial_grupo/tarea_Nero_new.py`:
  - Usa ControladorSE3 (DLS + espacio nulo + orientación R_PINZA_ABAJO_Y con error < 1 mm).
  - Compatible con simulación (`mode:=sim`) y brazo real (`mode:=real`) sin cambiar código:
      * Sensor 1D Arduino:  /sensor/cube_distance (std_msgs/Float64, m o cm)
      * Posición 3D cubo:   /cube/position        (geometry_msgs/PointStamped)
      * Fallback MuJoCo:    cube_joint en /feedback/joint_states
  - Parámetro `z_offset_seguridad` (default 0.0 m) para pruebas seguras elevadas en el lab.
  - Verificación automática de agarre y reporte de tasa de éxito acumulada.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from nero_ctrl.comun.cinematica_se3 import (
    ARM_JOINTS,
    ControladorSE3,
    R_PINZA_ABAJO_Y,
    resolve_scene_path,
    smoothstep,
)


CUBE_XML_ORIGIN_X = -0.28
CUBE_XML_ORIGIN_Y = -0.25
CUBE_XML_ORIGIN_Z = 0.084


class RecogerCuboCintaNode(Node):
    def __init__(self):
        super().__init__("tarea1_2_recoger_cinta")

        # Parámetros configurables (simulación y laboratorio)
        self.declare_parameter("scene_name", "scene_cube.xml")
        self.declare_parameter("cube_distance_topic", "/sensor/cube_distance")
        self.declare_parameter("cube_point_topic", "/cube/position")
        self.declare_parameter("distance_unit", "m")
        self.declare_parameter("sensor_y_origin", -0.25)
        self.declare_parameter("sensor_direction", 1.0)

        self.declare_parameter("cube_x", CUBE_XML_ORIGIN_X)
        self.declare_parameter("cube_z", CUBE_XML_ORIGIN_Z)
        self.declare_parameter("z_offset_seguridad", 0.0)
        self.declare_parameter("y_espera", 0.0)
        self.declare_parameter("y_captura_min", -0.26)
        self.declare_parameter("y_captura_max", 0.24)
        self.declare_parameter("lead_time", 0.22)
        self.declare_parameter("approach_height", 0.11)
        self.declare_parameter("lift_height", 0.12)
        self.declare_parameter("grasp_duration", 1.4)
        self.declare_parameter("lift_duration", 1.8)
        self.declare_parameter("gripper_abierto", 0.082)
        self.declare_parameter("gripper_cerrado", 0.008)
        self.declare_parameter("stop_conveyor_on_grasp", True)
        self.declare_parameter("depositar_y_repetir", False)
        self.declare_parameter("drop_x", -0.22)
        self.declare_parameter("drop_y", 0.18)
        self.declare_parameter("drop_z", 0.11)

        scene_path = resolve_scene_path(str(self.get_parameter("scene_name").value))
        self.z_offset_seguridad = float(self.get_parameter("z_offset_seguridad").value)
        self.ctrl = ControladorSE3(
            model_path=scene_path,
            damping=1.5e-2,
            alpha=0.35,
            max_step_norm=0.12,
            nullspace_gain=0.04,
            z_min_seguridad=0.072 + self.z_offset_seguridad,
        )

        self.distance_unit = str(self.get_parameter("distance_unit").value).strip().lower()
        self.sensor_y_origin = float(self.get_parameter("sensor_y_origin").value)
        self.sensor_direction = float(self.get_parameter("sensor_direction").value)

        self.cube_x = float(self.get_parameter("cube_x").value)
        self.cube_z = float(self.get_parameter("cube_z").value)
        self.y_espera = float(self.get_parameter("y_espera").value)
        self.y_captura_min = float(self.get_parameter("y_captura_min").value)
        self.y_captura_max = float(self.get_parameter("y_captura_max").value)
        self.lead_time = float(self.get_parameter("lead_time").value)
        self.approach_height = float(self.get_parameter("approach_height").value)
        self.lift_height = float(self.get_parameter("lift_height").value)
        self.grasp_duration = float(self.get_parameter("grasp_duration").value)
        self.lift_duration = float(self.get_parameter("lift_duration").value)
        self.gripper_abierto = float(self.get_parameter("gripper_abierto").value)
        self.gripper_cerrado = float(self.get_parameter("gripper_cerrado").value)
        self.stop_conveyor_on_grasp = bool(self.get_parameter("stop_conveyor_on_grasp").value)
        self.depositar_y_repetir = bool(self.get_parameter("depositar_y_repetir").value)
        self.drop_pos = np.array([
            float(self.get_parameter("drop_x").value),
            float(self.get_parameter("drop_y").value),
            float(self.get_parameter("drop_z").value) + self.z_offset_seguridad,
        ], dtype=float)

        # Estado del brazo y del cubo
        self.current_q = None
        self.theta_cmd = None
        self.gripper_meas = self.gripper_abierto
        self.gripper_cmd = self.gripper_abierto

        self.cube_pos = np.array([self.cube_x, self.sensor_y_origin, self.cube_z], dtype=float)
        self.cube_z_sim_lift = 0.0
        self.cube_vel_y = 0.03
        self.last_cube_y = None
        self.last_cube_t = None
        self.cube_detected = False
        self.has_dedicated_cube = False

        # Máquina de estados
        self.estado = "ESPERA"
        self.estado_inicio = time.monotonic()
        self.grasp_pos = None
        self.transition_start_pos = None
        self.conveyor_stopped = False

        # Métricas de desempeño
        self.intentos = 0
        self.exitos = 0
        self.last_log_t = 0.0
        self.start_t = time.monotonic()

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.pub_move_j = self.create_publisher(JointState, "control/move_j", 10)
        self.pub_direct = self.create_publisher(JointState, "control/joint_states", 10)

        self.create_subscription(JointState, "feedback/joint_states", self._on_feedback, qos)
        self.create_subscription(
            Float64, str(self.get_parameter("cube_distance_topic").value), self._on_cube_distance, qos
        )
        self.create_subscription(
            PointStamped, str(self.get_parameter("cube_point_topic").value), self._on_cube_point, qos
        )

        self.create_timer(0.04, self._control_loop)  # 25 Hz
        self.get_logger().info(
            f"Tarea 1.2 lista (z_offset_seguridad={self.z_offset_seguridad:+.3f} m, "
            f"depositar_y_repetir={self.depositar_y_repetir}). Esperando feedback..."
        )

    # ------------------------------------------------------------------ Callbacks
    def _update_cube_measurement(self, x: float, y: float, z: float):
        now = time.monotonic()
        if self.last_cube_y is not None and self.last_cube_t is not None:
            dt = now - self.last_cube_t
            if 0.005 < dt < 0.5:
                v_raw = (y - self.last_cube_y) / dt
                # Filtro anti-picos para sensores reales (ultrasónico/ToF)
                if abs(v_raw) < 0.35:
                    self.cube_vel_y = 0.80 * self.cube_vel_y + 0.20 * v_raw
        self.cube_pos[:] = [x, y, z]
        self.last_cube_y = y
        self.last_cube_t = now
        self.cube_detected = True

    def _on_feedback(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in ARM_JOINTS):
            self.current_q = np.array([pos[j] for j in ARM_JOINTS], dtype=float)
            if self.theta_cmd is None:
                self.theta_cmd = self.current_q.copy()
        if "gripper" in pos:
            self.gripper_meas = float(pos["gripper"])
        if "cube_z_joint" in pos:
            self.cube_z_sim_lift = float(pos["cube_z_joint"])

        # Fallback simulación MuJoCo si no llegan tópicos dedicados
        if not self.has_dedicated_cube and "cube_joint" in pos:
            cx = CUBE_XML_ORIGIN_X + float(pos.get("cube_x_joint", 0.0))
            cy = CUBE_XML_ORIGIN_Y + float(pos["cube_joint"])
            cz = CUBE_XML_ORIGIN_Z + float(pos.get("cube_z_joint", 0.0))
            self._update_cube_measurement(cx, cy, cz)

    def _on_cube_distance(self, msg: Float64):
        dist = float(msg.data)
        if self.distance_unit == "cm":
            dist *= 0.01
        y_world = self.sensor_y_origin + self.sensor_direction * dist
        self.has_dedicated_cube = True
        self._update_cube_measurement(self.cube_x, y_world, self.cube_z)

    def _on_cube_point(self, msg: PointStamped):
        self.has_dedicated_cube = True
        self._update_cube_measurement(float(msg.point.x), float(msg.point.y), float(msg.point.z))

    def _publish_move_j(self):
        msg = JointState()
        msg.name = ARM_JOINTS + ["gripper"]
        msg.position = self.theta_cmd.tolist() + [float(self.gripper_cmd)]
        msg.effort = [0.0] * 7 + [1.5]
        self.pub_move_j.publish(msg)

    def _set_conveyor_speed(self, speed: float):
        msg = JointState()
        msg.name = ["cube_motor"]
        msg.position = [float(speed)]
        self.pub_direct.publish(msg)

    def _set_state(self, nuevo_estado: str, pos_actual: np.ndarray):
        self.estado = nuevo_estado
        self.estado_inicio = time.monotonic()
        self.transition_start_pos = pos_actual.copy()
        self.get_logger().info(f"[Tarea 1.2] Estado -> {nuevo_estado}")

    # --------------------------------------------------------------- Bucle 25 Hz
    def _control_loop(self):
        if self.current_q is None or self.theta_cmd is None:
            return

        now = time.monotonic()
        dt_state = now - self.estado_inicio
        pos_actual, _ = self.ctrl.fk(self.current_q)

        # Predicción del cubo en Y
        y_pred = float(np.clip(
            self.cube_pos[1] + self.cube_vel_y * self.lead_time,
            self.y_captura_min,
            self.y_captura_max,
        ))
        z_grasp = self.cube_z + self.z_offset_seguridad

        # Posición objetivo por defecto (espera segura sobre la cinta)
        xd = np.array([self.cube_x, self.y_espera, z_grasp + self.approach_height], dtype=float)

        if self.estado == "ESPERA":
            self.gripper_cmd = self.gripper_abierto
            self.conveyor_stopped = False
            if self.cube_detected and (self.y_captura_min <= self.cube_pos[1] <= self.y_captura_max - 0.04):
                self._set_state("ACERCAMIENTO", pos_actual)

        elif self.estado == "ACERCAMIENTO":
            xd = np.array([self.cube_pos[0], y_pred, z_grasp + self.approach_height], dtype=float)
            self.gripper_cmd = self.gripper_abierto
            err_xy = float(np.linalg.norm(xd[:2] - pos_actual[:2]))
            err_z = abs(xd[2] - pos_actual[2])
            if err_xy <= 0.035 and err_z <= 0.04:
                self._set_state("BAJAR", pos_actual)

        elif self.estado == "BAJAR":
            # Sigue prediciendo Y mientras desciende para centrar el cubo entre los dedos
            y_des = float(np.clip(
                self.cube_pos[1] + self.cube_vel_y * (self.lead_time * 0.6),
                self.y_captura_min,
                self.y_captura_max,
            ))
            xd = np.array([self.cube_pos[0], y_des, z_grasp], dtype=float)
            self.gripper_cmd = self.gripper_abierto
            dist_3d = float(np.linalg.norm(np.array([self.cube_pos[0], self.cube_pos[1], z_grasp]) - pos_actual))
            if dist_3d <= 0.022 or (abs(pos_actual[2] - z_grasp) <= 0.012 and abs(pos_actual[1] - self.cube_pos[1]) <= 0.025):
                self.grasp_pos = np.array([self.cube_pos[0], self.cube_pos[1], z_grasp], dtype=float)
                self._set_state("AGARRAR", pos_actual)

        elif self.estado == "AGARRAR":
            self.gripper_cmd = self.gripper_cerrado
            if self.stop_conveyor_on_grasp and not self.conveyor_stopped:
                self._set_conveyor_speed(0.0)
                self.conveyor_stopped = True
                self.grasp_pos = np.array([self.cube_pos[0], self.cube_pos[1], z_grasp], dtype=float)
            elif not self.stop_conveyor_on_grasp and dt_state < 0.30:
                # Si la cinta física no se detiene, acompaña ligeramente durante el cierre inicial
                self.grasp_pos[1] = float(np.clip(self.cube_pos[1], self.y_captura_min, self.y_captura_max))
            xd = self.grasp_pos.copy()

            if dt_state >= self.grasp_duration:
                self._set_state("SUBIR", pos_actual)

        elif self.estado == "SUBIR":
            s = smoothstep(dt_state / max(self.lift_duration, 0.2))
            xd = self.grasp_pos.copy()
            xd[2] = self.grasp_pos[2] + self.lift_height * s
            self.gripper_cmd = self.gripper_cerrado

            if dt_state >= self.lift_duration + 0.3:
                self.intentos += 1
                # Éxito: en sim elevó el cubo >= 4 cm; en real el gripper mide ~4 cm (cubo sujeto)
                exito_sim = (not self.has_dedicated_cube) and (self.cube_z_sim_lift >= 0.04)
                exito_real = (0.020 <= self.gripper_meas <= 0.058)
                exito = exito_sim or (self.has_dedicated_cube and exito_real)
                if exito:
                    self.exitos += 1
                tasa = 100.0 * self.exitos / max(self.intentos, 1)
                self.get_logger().info(
                    f"[Tarea 1.2] Intento #{self.intentos}: {'EXITO' if exito else 'FALLO'} "
                    f"(lift_sim={self.cube_z_sim_lift:.3f} m, gripper={self.gripper_meas:.3f} m) | "
                    f"Tasa acumulada: {self.exitos}/{self.intentos} ({tasa:.1f}%)"
                )
                if self.depositar_y_repetir:
                    self._set_state("DEPOSITAR", pos_actual)
                else:
                    self._set_state("COMPLETADO", pos_actual)

        elif self.estado == "DEPOSITAR":
            s = smoothstep(dt_state / 2.0)
            target_high = self.drop_pos.copy()
            target_high[2] = self.grasp_pos[2] + self.lift_height
            xd = (1.0 - s) * self.transition_start_pos + s * target_high
            self.gripper_cmd = self.gripper_cerrado
            if dt_state >= 2.2:
                self._set_state("SOLTAR", pos_actual)

        elif self.estado == "SOLTAR":
            xd = self.drop_pos.copy()
            self.gripper_cmd = self.gripper_abierto if dt_state >= 0.8 else self.gripper_cerrado
            if dt_state >= 1.6:
                self._set_state("ESPERA", pos_actual)

        elif self.estado == "COMPLETADO":
            xd = self.grasp_pos.copy()
            xd[2] = self.grasp_pos[2] + self.lift_height
            self.gripper_cmd = self.gripper_cerrado

        # Paso diferencial en SE(3) + publicación en /control/move_j
        self.theta_cmd, pos_actual, err = self.ctrl.step_ik(
            self.current_q, self.theta_cmd, xd, R_PINZA_ABAJO_Y
        )
        self._publish_move_j()

        t = now - self.start_t
        if t - self.last_log_t >= 1.0:
            self.get_logger().info(
                f"[{self.estado}] cubo=({self.cube_pos[0]:+.3f},{self.cube_pos[1]:+.3f},{self.cube_pos[2]:+.3f}) "
                f"vy={self.cube_vel_y:+.3f} m/s | ee_err={err:.4f} m | gripper={self.gripper_meas:.3f} m"
            )
            self.last_log_t = t


def main(args=None):
    rclpy.init(args=args)
    node = RecogerCuboCintaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
controller_node.py — Controlador Reactivo Puro de Evasión de Obstáculos (Parte 2)
=================================================================================
Nodo oficial de control de `nero_ctrl`. Su única responsabilidad es el control
reactivo en lazo cerrado en espacio cartesiano (SE(3)):
  - Recibe una referencia nominal de cualquier nodo de tarea en `/control/nominal_target`:
      Float64MultiArray: [x_nom, y_nom, z_nom, vx_nom, vy_nom, vz_nom, gripper]
  - Escucha el estado real del brazo (`/feedback/joint_states`) y los cubos en
    movimiento (`/cubes/positions`, `/sensor/cube_distance`, `/cube/position` o
    fallback MuJoCo `cube*_joint`).
  - Evalúa la distancia mínima predictiva por velocidad relativa
      v_rel = v_y_cubo - v_y_brazo  en  tau in [0, t_anticipacion]
    y aplica elevación suave asimétrica (`tau_subida` rápido, `tau_bajada` suave)
    sobre `z_nom` antes de resolver la IK convergente en SE(3) hacia `/control/move_j`.

Es completamente independiente de la tarea: funciona igual para `tarea2_1_estatica`
(vy_nom = 0), `tarea2_2_movil` (patrullaje a velocidad constante) o cualquier futura tarea.
"""

import math
import time
from typing import Dict, Tuple

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PointStamped, PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

from nero_ctrl.comun.cinematica_se3 import (
    ARM_JOINTS,
    ControladorSE3,
    R_PINZA_ABAJO_Y,
    resolve_scene_path,
    smoothstep,
)


class ControllerNode(Node):
    """Controlador reactivo puro de evasión predictiva en SE(3)."""

    def __init__(self):
        super().__init__("controller_node")

        # ------------------------------------------------------------------
        # Parámetros del controlador (valores ajustados)
        # ------------------------------------------------------------------
        self.declare_parameter("scene_name", "scene_evasion_cinta.xml")
        self.declare_parameter("nominal_target_topic", "/control/nominal_target")

        # Parámetros de evasión vertical y activación
        self.declare_parameter("delta_z_evasion", 0.05)       # elevación adicional al evadir (+5 cm)
        self.declare_parameter("z_offset_seguridad", 0.0)     # margen extra para pruebas seguras en lab
        self.declare_parameter("radio_alerta", 0.085)         # distancia horizontal donde inicia evasión (m)
        self.declare_parameter("radio_pleno", 0.065)          # distancia donde alcanza 100% de elevación (m)
        self.declare_parameter("t_anticipacion", 0.32)        # horizonte predictivo (s)
        self.declare_parameter("tau_subida", 0.04)            # constante de tiempo ascenso rápido (s)
        self.declare_parameter("tau_bajada", 0.22)            # constante de tiempo retorno suave (s)

        # Geometría de la cinta para interpretar sensores 1D / fallback MuJoCo
        self.declare_parameter("x_cinta", -0.28)
        self.declare_parameter("cube_z_default", 0.084)
        self.declare_parameter("distance_unit", "m")          # 'm' o 'cm'
        self.declare_parameter("sensor_y_origin", -0.25)
        self.declare_parameter("sensor_direction", 1.0)

        scene_path = resolve_scene_path(str(self.get_parameter("scene_name").value))
        target_topic = str(self.get_parameter("nominal_target_topic").value)

        self.delta_z = float(self.get_parameter("delta_z_evasion").value)
        self.z_offset = float(self.get_parameter("z_offset_seguridad").value)
        self.r_alerta = float(self.get_parameter("radio_alerta").value)
        self.r_pleno = float(self.get_parameter("radio_pleno").value)
        self.t_antic = float(self.get_parameter("t_anticipacion").value)
        self.tau_subida = max(float(self.get_parameter("tau_subida").value), 0.01)
        self.tau_bajada = max(float(self.get_parameter("tau_bajada").value), 0.02)

        self.x_cinta = float(self.get_parameter("x_cinta").value)
        self.cube_z_default = float(self.get_parameter("cube_z_default").value)
        self.distance_unit = str(self.get_parameter("distance_unit").value).strip().lower()
        self.sensor_y_origin = float(self.get_parameter("sensor_y_origin").value)
        self.sensor_direction = float(self.get_parameter("sensor_direction").value)

        self.ctrl = ControladorSE3(
            model_path=scene_path,
            damping=1.5e-2,
            alpha=0.25,
            max_step_norm=0.12,
            nullspace_gain=0.04,
            z_min_seguridad=0.075 + self.z_offset,
        )

        # Estado del robot y de la referencia nominal recibida desde el nodo de tarea
        self.current_q = None
        self.theta_cmd = None
        self.nominal_pos = None     # [x, y, z]
        self.nominal_vel = None     # [vx, vy, vz]
        self.gripper_cmd = 0.010

        # Registro de obstáculos: id -> (pos_xyz, vel_y, timestamp)
        self.obstaculos: Dict[str, Tuple[np.ndarray, float, float]] = {}
        self.has_dedicated_obs = False

        # Estado interno del filtro de evasión
        self.sigma_actual = 0.0
        self.evadiendo = False
        self.evasiones_contadas = 0
        self.last_ctrl_t = time.monotonic()
        self.last_log_t = 0.0

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.pub_move_j = self.create_publisher(JointState, "control/move_j", 10)

        self.create_subscription(Float64MultiArray, target_topic, self._on_nominal_target, qos)
        self.create_subscription(JointState, "feedback/joint_states", self._on_feedback, qos)
        self.create_subscription(PoseArray, "/cubes/positions", self._on_cubes_array, qos)
        self.create_subscription(Float64, "/sensor/cube_distance", self._on_cube_distance, qos)
        self.create_subscription(PointStamped, "/cube/position", self._on_cube_point, qos)

        # Bucle de control reactivo a 40 Hz
        self.create_timer(0.025, self._control_loop)

        self.get_logger().info(
            f"[controller_node] Controlador puro listo | escuchando '{target_topic}' | "
            f"delta_z={self.delta_z:.3f} m, r_alerta={self.r_alerta:.3f} m, "
            f"r_pleno={self.r_pleno:.3f} m, t_antic={self.t_antic:.2f} s"
        )

    # -------------------------------------------------------------- Suscriptores
    def _on_nominal_target(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            return
        self.nominal_pos = np.array(msg.data[0:3], dtype=float)
        if len(msg.data) >= 6:
            self.nominal_vel = np.array(msg.data[3:6], dtype=float)
        else:
            self.nominal_vel = np.zeros(3, dtype=float)
        if len(msg.data) >= 7:
            self.gripper_cmd = float(msg.data[6])

    def _update_obstacle(self, obs_id: str, x: float, y: float, z: float):
        now = time.monotonic()
        vel_y = 0.05
        if obs_id in self.obstaculos:
            prev_pos, prev_vy, prev_t = self.obstaculos[obs_id]
            dt = now - prev_t
            if 0.005 < dt < 0.5:
                v_raw = (y - prev_pos[1]) / dt
                vel_y = (0.80 * prev_vy + 0.20 * v_raw) if abs(v_raw) < 0.40 else prev_vy
            else:
                vel_y = prev_vy
        self.obstaculos[obs_id] = (np.array([x, y, z], dtype=float), vel_y, now)

    def _on_feedback(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in ARM_JOINTS):
            self.current_q = np.array([pos[j] for j in ARM_JOINTS], dtype=float)
            if self.theta_cmd is None:
                self.theta_cmd = self.current_q.copy()

        if self.has_dedicated_obs:
            return

        for name, val in pos.items():
            if name.startswith("cube") and name.endswith("_joint") and "_x_" not in name and "_z_" not in name:
                y_w = (-0.25 + float(val)) if name == "cube_joint" else float(val)
                self._update_obstacle(name, self.x_cinta, y_w, self.cube_z_default)

    def _on_cubes_array(self, msg: PoseArray):
        self.has_dedicated_obs = True
        for idx, pose in enumerate(msg.poses):
            self._update_obstacle(
                f"pose_{idx}", float(pose.position.x), float(pose.position.y), float(pose.position.z)
            )

    def _on_cube_distance(self, msg: Float64):
        dist = float(msg.data)
        if self.distance_unit == "cm":
            dist *= 0.01
        self.has_dedicated_obs = True
        y_w = self.sensor_y_origin + self.sensor_direction * dist
        self._update_obstacle("sensor_1d", self.x_cinta, y_w, self.cube_z_default)

    def _on_cube_point(self, msg: PointStamped):
        self.has_dedicated_obs = True
        self._update_obstacle(
            "point_3d", float(msg.point.x), float(msg.point.y), float(msg.point.z)
        )

    # ---------------------------------------------------- Ley de Control Reactivo
    def _evaluar_evasion_relativa(self, arm_xy: np.ndarray, arm_vy: float) -> Tuple[float, float]:
        """Distancia mínima predictiva por velocidad relativa v_rel = v_y_cubo - v_y_brazo."""
        now = time.monotonic()
        sigma_max = 0.0
        min_d_now = 999.0

        stale_ids = []
        for oid, (p_obs, obs_vy, ts) in self.obstaculos.items():
            if now - ts > 1.5:
                stale_ids.append(oid)
                continue

            dx = p_obs[0] - arm_xy[0]
            dy0 = p_obs[1] - arm_xy[1]
            v_rel = obs_vy - arm_vy
            min_d_now = min(min_d_now, float(np.hypot(dx, dy0)))

            if abs(v_rel) > 1e-5:
                tau_star = float(np.clip(-dy0 / v_rel, 0.0, self.t_antic))
            else:
                tau_star = 0.0

            dy_min = abs(dy0 + v_rel * tau_star)
            dy_eff = max(min(abs(dy0), dy_min) - 0.020, 0.0)
            d_eff = float(np.hypot(dx, dy_eff))

            denom = max(self.r_alerta - self.r_pleno, 1e-3)
            s = float(np.clip((self.r_alerta - d_eff) / denom, 0.0, 1.0))
            sigma_max = max(sigma_max, smoothstep(s))

        for oid in stale_ids:
            self.obstaculos.pop(oid, None)

        return sigma_max, min_d_now

    def _control_loop(self):
        if self.current_q is None or self.theta_cmd is None or self.nominal_pos is None:
            return

        now = time.monotonic()
        dt = max(min(now - self.last_ctrl_t, 0.1), 1e-3)
        self.last_ctrl_t = now

        pos_actual, _ = self.ctrl.fk(self.current_q)
        x_nom, y_nom, z_nom = self.nominal_pos
        vy_nom = float(self.nominal_vel[1])

        sigma_nom, d_nom = self._evaluar_evasion_relativa(np.array([x_nom, y_nom]), vy_nom)
        sigma_cur, d_cur = self._evaluar_evasion_relativa(pos_actual[:2], vy_nom)
        sigma_target = max(sigma_nom, sigma_cur)
        min_d = min(d_nom, d_cur)

        tau = self.tau_subida if sigma_target > self.sigma_actual else self.tau_bajada
        beta = 1.0 - math.exp(-dt / tau)
        self.sigma_actual += beta * (sigma_target - self.sigma_actual)

        if not self.evadiendo and self.sigma_actual > 0.55:
            self.evadiendo = True
            self.evasiones_contadas += 1
            self.get_logger().info(
                f"[controller_node] Evasión #{self.evasiones_contadas} activada "
                f"(Y_ee={pos_actual[1]:+.3f}, vy_ref={vy_nom:+.3f}, d_xy={min_d:.3f} m)"
            )
        elif self.evadiendo and self.sigma_actual < 0.10:
            self.evadiendo = False

        xd = np.array([
            x_nom,
            y_nom,
            z_nom + self.z_offset + self.delta_z * self.sigma_actual,
        ], dtype=float)

        self.theta_cmd, _, _ = self.ctrl.step_ik(
            self.current_q, self.theta_cmd, xd, R_PINZA_ABAJO_Y
        )

        msg = JointState()
        msg.name = ARM_JOINTS + ["gripper"]
        msg.position = self.theta_cmd.tolist() + [self.gripper_cmd]
        msg.effort = [0.0] * 7 + [1.0]
        self.pub_move_j.publish(msg)

        if now - self.last_log_t >= 1.5:
            self.get_logger().info(
                f"[controller_node] obs={len(self.obstaculos)} | Y_ee={pos_actual[1]:+.3f} (ref={y_nom:+.3f}) | "
                f"Z_ee={pos_actual[2]:.3f} m | sigma={self.sigma_actual:.2f} | evasiones={self.evasiones_contadas}"
            )
            self.last_log_t = now


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

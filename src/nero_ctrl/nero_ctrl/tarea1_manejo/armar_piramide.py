#!/usr/bin/env python3
"""
armar_piramide.py — Tarea 1.3 (Entrega Final)
=============================================
Arma automáticamente una pirámide 3-2-1 con 6 cubos de 4 cm:
  - Base (piso 1):       3 cubos alineados en X (separación 4.4 cm -> 4 mm holgura)
  - Piso intermedio (2): 2 cubos centrados entre los pares de la base
  - Punta (piso 3):      1 cubo centrado en la cima

Ventaja geométrica clave:
  Usa `R_PINZA_ABAJO_Y` (dedos abriendo a lo largo de Y), mientras la fila de la pirámide
  está orientada a lo largo de X. Así, los dedos sujetan las caras +Y/-Y de cada cubo y
  NUNCA colisionan con los cubos vecinos ya colocados en la misma fila.

Compatible con simulación (`mode:=sim`, escena `scene_piramide.xml`) y brazo real (`mode:=real`),
con `z_offset_seguridad` para validar toda la secuencia en el aire antes de bajar a la mesa.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState

from nero_ctrl.comun.cinematica_se3 import (
    ARM_JOINTS,
    ControladorSE3,
    R_PINZA_ABAJO_Y,
    resolve_scene_path,
    smootherstep,
)


class ArmarPiramideNode(Node):
    def __init__(self):
        super().__init__("tarea1_3_piramide")

        # Parámetros configurables para calibrar en el laboratorio
        self.declare_parameter("scene_name", "scene_piramide.xml")
        self.declare_parameter("z_mesa", 0.064)
        self.declare_parameter("lado_cubo", 0.040)
        self.declare_parameter("espaciado_x", 0.044)
        self.declare_parameter("piramide_x", -0.300)
        self.declare_parameter("piramide_y", 0.000)
        self.declare_parameter("z_offset_seguridad", 0.0)
        self.declare_parameter("altura_transito", 0.245)
        self.declare_parameter("gripper_abierto", 0.080)
        self.declare_parameter("gripper_cerrado", 0.006)
        self.declare_parameter("velocidad_factor", 1.0)

        scene_path = resolve_scene_path(str(self.get_parameter("scene_name").value))
        self.z_mesa = float(self.get_parameter("z_mesa").value)
        self.lado = float(self.get_parameter("lado_cubo").value)
        self.espaciado = float(self.get_parameter("espaciado_x").value)
        self.pir_x = float(self.get_parameter("piramide_x").value)
        self.pir_y = float(self.get_parameter("piramide_y").value)
        self.z_offset = float(self.get_parameter("z_offset_seguridad").value)
        self.z_transito = float(self.get_parameter("altura_transito").value) + self.z_offset
        self.gripper_abierto = float(self.get_parameter("gripper_abierto").value)
        self.gripper_cerrado = float(self.get_parameter("gripper_cerrado").value)
        self.vfac = max(float(self.get_parameter("velocidad_factor").value), 0.25)

        self.ctrl = ControladorSE3(
            model_path=scene_path,
            damping=1.2e-2,
            alpha=0.38,
            max_step_norm=0.11,
            nullspace_gain=0.04,
            z_min_seguridad=self.z_mesa + 0.010 + self.z_offset,
        )

        # Definición de los 6 pares (origen_pick, destino_piramide)
        z1 = self.z_mesa + 0.5 * self.lado + self.z_offset          # 0.084
        z2 = self.z_mesa + 1.5 * self.lado + self.z_offset          # 0.124
        z3 = self.z_mesa + 2.5 * self.lado + self.z_offset          # 0.164
        s = self.espaciado

        self.plan_cubos = [
            # (nombre, pos_pick, pos_place)
            ("Piso1-Izq",   np.array([-0.24, -0.16, z1]), np.array([self.pir_x + s,       self.pir_y, z1])),
            ("Piso1-Cen",   np.array([-0.30, -0.16, z1]), np.array([self.pir_x,           self.pir_y, z1])),
            ("Piso1-Der",   np.array([-0.36, -0.16, z1]), np.array([self.pir_x - s,       self.pir_y, z1])),
            ("Piso2-Izq",   np.array([-0.24, +0.16, z1]), np.array([self.pir_x + 0.5 * s, self.pir_y, z2])),
            ("Piso2-Der",   np.array([-0.30, +0.16, z1]), np.array([self.pir_x - 0.5 * s, self.pir_y, z2])),
            ("Piso3-Punta", np.array([-0.36, +0.16, z1]), np.array([self.pir_x,           self.pir_y, z3])),
        ]

        # Estado del controlador
        self.current_q = None
        self.theta_cmd = None
        self.gripper_cmd = self.gripper_abierto

        self.cubo_idx = 0
        self.fase = "INIT"
        self.fase_t0 = time.monotonic()
        self.p_inicio = None
        self.p_fin = None
        self.fase_dur = 1.0
        self.last_log_t = 0.0

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.pub_move_j = self.create_publisher(JointState, "control/move_j", 10)
        self.create_subscription(JointState, "feedback/joint_states", self._on_feedback, qos)
        self.create_timer(0.04, self._control_loop)  # 25 Hz

        self.get_logger().info(
            f"Tarea 1.3 (Pirámide 3-2-1) iniciada. Centro=({self.pir_x:.3f}, {self.pir_y:.3f}), "
            f"z_mesa={self.z_mesa:.3f} m, z_offset_seguridad={self.z_offset:+.3f} m."
        )

    def _on_feedback(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in ARM_JOINTS):
            self.current_q = np.array([pos[j] for j in ARM_JOINTS], dtype=float)
            if self.theta_cmd is None:
                self.theta_cmd = self.current_q.copy()

    def _publish_move_j(self):
        msg = JointState()
        msg.name = ARM_JOINTS + ["gripper"]
        msg.position = self.theta_cmd.tolist() + [float(self.gripper_cmd)]
        msg.effort = [0.0] * 7 + [1.5]
        self.pub_move_j.publish(msg)

    def _start_phase(self, fase: str, p_from: np.ndarray, p_to: np.ndarray, dur_s: float, gripper: float):
        self.fase = fase
        self.fase_t0 = time.monotonic()
        self.p_inicio = p_from.copy()
        self.p_fin = p_to.copy()
        self.fase_dur = max(dur_s / self.vfac, 0.25)
        self.gripper_cmd = gripper
        if self.cubo_idx < len(self.plan_cubos):
            etiqueta = self.plan_cubos[self.cubo_idx][0]
            self.get_logger().info(f"[Cubo {self.cubo_idx + 1}/6 - {etiqueta}] Fase -> {fase}")

    def _control_loop(self):
        if self.current_q is None or self.theta_cmd is None:
            return

        now = time.monotonic()
        pos_actual, _ = self.ctrl.fk(self.current_q)

        # Arranque inicial desde la pose actual hacia el primer cubo
        if self.fase == "INIT":
            _, pick, _ = self.plan_cubos[0]
            p_hover = np.array([pick[0], pick[1], self.z_transito], dtype=float)
            self._start_phase("IR_SOBRE_PICK", pos_actual, p_hover, 2.2, self.gripper_abierto)

        if self.fase == "COMPLETADO":
            xd = self.p_fin.copy()
            self.theta_cmd, _, _ = self.ctrl.step_ik(self.current_q, self.theta_cmd, xd, R_PINZA_ABAJO_Y)
            self._publish_move_j()
            return

        # Interpolación minimum-jerk dentro de la fase actual
        elapsed = now - self.fase_t0
        s = smootherstep(elapsed / self.fase_dur)
        xd = (1.0 - s) * self.p_inicio + s * self.p_fin

        self.theta_cmd, pos_actual, err_target = self.ctrl.step_ik(
            self.current_q, self.theta_cmd, xd, R_PINZA_ABAJO_Y
        )
        self._publish_move_j()

        err_fin = float(np.linalg.norm(self.p_fin - pos_actual))
        done_move = (elapsed >= self.fase_dur and err_fin <= 0.008) or (elapsed >= self.fase_dur + 1.4)
        done_wait = elapsed >= self.fase_dur

        etiqueta, pick, place = self.plan_cubos[self.cubo_idx]
        z_safe = max(self.z_transito, place[2] + 0.085)

        if self.fase == "IR_SOBRE_PICK" and done_move:
            self._start_phase("BAJAR_PICK", pos_actual, pick, 1.6, self.gripper_abierto)

        elif self.fase == "BAJAR_PICK" and done_move:
            self._start_phase("CERRAR_PICK", pick, pick, 0.9, self.gripper_cerrado)

        elif self.fase == "CERRAR_PICK" and done_wait:
            p_up = np.array([pick[0], pick[1], z_safe], dtype=float)
            self._start_phase("SUBIR_PICK", pos_actual, p_up, 1.5, self.gripper_cerrado)

        elif self.fase == "SUBIR_PICK" and done_move:
            p_over_place = np.array([place[0], place[1], z_safe], dtype=float)
            self._start_phase("TRANSITO_PLACE", pos_actual, p_over_place, 1.9, self.gripper_cerrado)

        elif self.fase == "TRANSITO_PLACE" and done_move:
            # 2 mm sobre la cota exacta para posado suave sin empujar la pirámide
            p_drop = place + np.array([0.0, 0.0, 0.002])
            self._start_phase("BAJAR_PLACE", pos_actual, p_drop, 1.9, self.gripper_cerrado)

        elif self.fase == "BAJAR_PLACE" and done_move:
            self._start_phase("ASENTAR_PLACE", self.p_fin, self.p_fin, 0.35, self.gripper_cerrado)

        elif self.fase == "ASENTAR_PLACE" and done_wait:
            self._start_phase("SOLTAR_PLACE", self.p_fin, self.p_fin, 0.75, self.gripper_abierto)

        elif self.fase == "SOLTAR_PLACE" and done_wait:
            p_retract = np.array([place[0], place[1], z_safe], dtype=float)
            self._start_phase("RETRAER_PLACE", pos_actual, p_retract, 1.4, self.gripper_abierto)

        elif self.fase == "RETRAER_PLACE" and done_move:
            self.get_logger().info(f"[Tarea 1.3] Cubo {self.cubo_idx + 1}/6 ({etiqueta}) colocado con éxito.")
            self.cubo_idx += 1
            if self.cubo_idx >= len(self.plan_cubos):
                p_final = np.array([self.pir_x, self.pir_y, self.z_transito + 0.02], dtype=float)
                self.fase = "COMPLETADO"
                self.p_fin = p_final
                self.gripper_cmd = self.gripper_abierto
                self.get_logger().info("[Tarea 1.3] === PIRAMIDE 3-2-1 COMPLETADA (6/6 cubos) ===")
            else:
                _, next_pick, _ = self.plan_cubos[self.cubo_idx]
                p_next_hover = np.array([next_pick[0], next_pick[1], self.z_transito], dtype=float)
                self._start_phase("IR_SOBRE_PICK", pos_actual, p_next_hover, 1.8, self.gripper_abierto)

        if now - self.last_log_t >= 2.0:
            self.get_logger().info(
                f"[Tarea 1.3] Cubo {min(self.cubo_idx + 1, 6)}/6 | fase={self.fase} | err_fin={err_fin:.4f} m"
            )
            self.last_log_t = now


def main(args=None):
    rclpy.init(args=args)
    node = ArmarPiramideNode()
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

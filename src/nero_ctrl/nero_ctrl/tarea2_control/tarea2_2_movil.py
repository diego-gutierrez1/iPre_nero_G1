#!/usr/bin/env python3
"""
tarea2_2_movil.py — Nodo de Tarea 2.2 (Trayectoria a velocidad constante sobre la cinta)
========================================================================================
Define únicamente la tarea nominal de la Parte 2.2: desplazar el efector a velocidad
constante de ida y vuelta desde un extremo de la cinta (`y_min`) hasta el otro (`y_max`)
a baja altura (`z_bajo`), con un frenado/aceleración breve solo en los extremos para
evitar tirones mecánicos.

Publica `[x_nom, y_nom, z_nom, vx_nom, vy_nom, vz_nom, gripper]` en
`/control/nominal_target` para que `controller_node` aplique la evasión de cubos
en movimiento y la cinemática inversa.
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray


class Tarea22MovilNode(Node):
    def __init__(self):
        super().__init__("tarea2_2_movil")

        self.declare_parameter("target_topic", "/control/nominal_target")
        self.declare_parameter("x_cinta", -0.28)
        self.declare_parameter("y_min", -0.20)
        self.declare_parameter("y_max", 0.20)
        self.declare_parameter("velocidad_y", 0.10)    # velocidad lineal constante (m/s, mayor que la cinta para sobrepasos claros)
        self.declare_parameter("aceleracion_y", 0.40)  # rampa breve solo en los extremos (m/s^2)
        self.declare_parameter("z_bajo", 0.094)
        self.declare_parameter("gripper_width", 0.010)
        self.declare_parameter("rate_hz", 40.0)

        topic = str(self.get_parameter("target_topic").value)
        self.x_cinta = float(self.get_parameter("x_cinta").value)
        self.y_min = float(self.get_parameter("y_min").value)
        self.y_max = float(self.get_parameter("y_max").value)
        self.v_crucero = max(abs(float(self.get_parameter("velocidad_y").value)), 0.01)
        self.a_extremo = max(abs(float(self.get_parameter("aceleracion_y").value)), 0.05)
        self.z_bajo = float(self.get_parameter("z_bajo").value)
        self.gripper_width = float(self.get_parameter("gripper_width").value)
        rate_hz = max(float(self.get_parameter("rate_hz").value), 5.0)

        # Estado de la trayectoria trapezoidal ida y vuelta
        self.y_pos = self.y_min
        self.v_cur = 0.0
        self.sentido = 1.0  # +1 hacia y_max, -1 hacia y_min
        self.last_t = time.monotonic()

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.pub_target = self.create_publisher(Float64MultiArray, topic, qos)
        self.create_timer(1.0 / rate_hz, self._publish_reference)

        self.get_logger().info(
            f"[Tarea 2.2] Publicando patrullaje a velocidad constante ({self.v_crucero:.3f} m/s) "
            f"entre Y=[{self.y_min:+.2f}, {self.y_max:+.2f}] m en '{topic}'."
        )

    def _publish_reference(self):
        now = time.monotonic()
        dt = max(min(now - self.last_t, 0.1), 1e-4)
        self.last_t = now

        # Distancia de frenado necesaria para invertir suavemente en el extremo
        d_frenado = (self.v_crucero ** 2) / (2.0 * self.a_extremo)
        dist_extremo = (self.y_max - self.y_pos) if self.sentido > 0 else (self.y_pos - self.y_min)

        if dist_extremo <= 0.002:
            # Invierte sentido limpiamente en el extremo
            self.sentido *= -1.0
            self.v_cur = 0.0
        elif dist_extremo <= d_frenado:
            # Rampa breve de desaceleración al llegar al extremo
            v_des = self.sentido * max((2.0 * self.a_extremo * dist_extremo) ** 0.5, 0.005)
            dv = v_des - self.v_cur
            max_dv = self.a_extremo * dt
            self.v_cur += max(-max_dv, min(max_dv, dv))
        else:
            # Acelera hasta v_crucero y luego mantiene velocidad estrictamente constante
            v_des = self.sentido * self.v_crucero
            dv = v_des - self.v_cur
            max_dv = self.a_extremo * dt
            self.v_cur += max(-max_dv, min(max_dv, dv))

        self.y_pos = max(self.y_min, min(self.y_max, self.y_pos + self.v_cur * dt))

        msg = Float64MultiArray()
        # Formato: [x_nom, y_nom, z_nom, vx_nom, vy_nom, vz_nom, gripper]
        msg.data = [
            self.x_cinta,
            self.y_pos,
            self.z_bajo,
            0.0,
            self.v_cur,
            0.0,
            self.gripper_width,
        ]
        self.pub_target.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Tarea22MovilNode()
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

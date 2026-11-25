#!/usr/bin/env python3
"""
tarea2_1_estatica.py — Nodo de Tarea 2.1 (Referencia fija sobre la cinta)
=========================================================================
Define únicamente la tarea nominal de la Parte 2.1: mantener el efector en una
posición fija a baja altura sobre la cinta transportadora (x_cinta, y_espera, z_bajo)
con velocidad nominal cero, y la publica en `/control/nominal_target` para que
`controller_node` se encargue de la evasión de cubos y la cinemática inversa.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray


class Tarea21EstaticaNode(Node):
    def __init__(self):
        super().__init__("tarea2_1_estatica")

        self.declare_parameter("target_topic", "/control/nominal_target")
        self.declare_parameter("x_cinta", -0.28)
        self.declare_parameter("y_espera", 0.00)
        self.declare_parameter("z_bajo", 0.094)       # bajo el tope del cubo (0.104 m)
        self.declare_parameter("gripper_width", 0.010)
        self.declare_parameter("rate_hz", 40.0)

        topic = str(self.get_parameter("target_topic").value)
        self.x_cinta = float(self.get_parameter("x_cinta").value)
        self.y_espera = float(self.get_parameter("y_espera").value)
        self.z_bajo = float(self.get_parameter("z_bajo").value)
        self.gripper_width = float(self.get_parameter("gripper_width").value)
        rate_hz = max(float(self.get_parameter("rate_hz").value), 5.0)

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.pub_target = self.create_publisher(Float64MultiArray, topic, qos)
        self.create_timer(1.0 / rate_hz, self._publish_reference)

        self.get_logger().info(
            f"[Tarea 2.1] Publicando referencia estática ({self.x_cinta:+.3f}, "
            f"{self.y_espera:+.3f}, {self.z_bajo:.3f}) m en '{topic}'."
        )

    def _publish_reference(self):
        msg = Float64MultiArray()
        # Formato: [x_nom, y_nom, z_nom, vx_nom, vy_nom, vz_nom, gripper]
        msg.data = [
            self.x_cinta,
            self.y_espera,
            self.z_bajo,
            0.0,
            0.0,
            0.0,
            self.gripper_width,
        ]
        self.pub_target.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Tarea21EstaticaNode()
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

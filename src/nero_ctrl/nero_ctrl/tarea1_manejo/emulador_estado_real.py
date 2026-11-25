#!/usr/bin/env python3
"""
emulador_estado_real.py — Nodo de prueba en casa para Tarea 1.1
===============================================================
Emula exactamente los tópicos que publicará el hardware real en el laboratorio:
  - /feedback/joint_states (sensor_msgs/JointState a 100 Hz): joint1..7 + gripper
  - /sensor/cube_distance  (std_msgs/Float64 a 30 Hz): distancia del cubo en la cinta (m)
    (o /cube/position en geometry_msgs/PointStamped si publish_mode:='point')

Sirve para validar visualmente `gemelo_digital_mujoco.py` sin necesidad de estar
conectado al brazo físico AgileX Nero ni al Arduino.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]


class EmuladorEstadoRealNode(Node):
    def __init__(self):
        super().__init__("emulador_estado_real")

        self.declare_parameter("cube_speed", 0.05)          # m/s
        self.declare_parameter("belt_length", 0.50)         # m (recorrido total sobre cinta)
        self.declare_parameter("publish_mode", "distance")  # 'distance' o 'point'

        self.cube_speed = float(self.get_parameter("cube_speed").value)
        self.belt_length = float(self.get_parameter("belt_length").value)
        self.publish_mode = str(self.get_parameter("publish_mode").value).strip().lower()

        qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        self.pub_joints = self.create_publisher(JointState, "/feedback/joint_states", qos)
        self.pub_dist = self.create_publisher(Float64, "/sensor/cube_distance", qos)
        self.pub_point = self.create_publisher(PointStamped, "/cube/position", qos)

        self.t0 = time.monotonic()
        self.create_timer(0.01, self._publish_arm)          # 100 Hz
        self.create_timer(1.0 / 30.0, self._publish_cube)   # 30 Hz

        self.get_logger().info(
            f"Emulador de estado real iniciado (modo cubo='{self.publish_mode}', "
            f"vel={self.cube_speed:.3f} m/s). Publicando /feedback/joint_states y cubo."
        )

    def _publish_arm(self):
        t = time.monotonic() - self.t0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_JOINTS + ["gripper"]
        # Movimiento articular suave y seguro para verificar los 7 GDL + pinza
        q = [
            0.35 * math.sin(0.6 * t),
            0.25 * math.sin(0.8 * t) - 0.25,
            0.20 * math.cos(0.7 * t),
            1.40 + 0.30 * math.sin(0.9 * t),
            0.40 * math.sin(1.1 * t),
            0.35 * math.cos(0.8 * t),
            0.50 * math.sin(1.2 * t),
        ]
        gripper_width = 0.04 + 0.035 * math.sin(1.5 * t)  # 0.005 .. 0.075 m
        msg.position = q + [gripper_width]
        msg.velocity = [0.0] * 8
        self.pub_joints.publish(msg)

    def _publish_cube(self):
        t = time.monotonic() - self.t0
        dist = (self.cube_speed * t) % max(self.belt_length, 0.05)

        if self.publish_mode == "point":
            pt = PointStamped()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.header.frame_id = "base_link"
            pt.point.x = -0.28
            pt.point.y = -0.25 + dist
            pt.point.z = 0.084
            self.pub_point.publish(pt)
        else:
            dmsg = Float64()
            dmsg.data = float(dist)
            self.pub_dist.publish(dmsg)


def main(args=None):
    rclpy.init(args=args)
    node = EmuladorEstadoRealNode()
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

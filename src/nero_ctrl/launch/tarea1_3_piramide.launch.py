"""
tarea1_3_piramide.launch.py
===========================
Lanza la Tarea 1.3 (armado automático de pirámide 3-2-1 con 6 cubos de 4 cm):
  - mode:=sim  (default): lanza `nero_sim_bridge` con `scene_piramide.xml` + `tarea1_3_piramide`.
  - mode:=real: lanza `agx_arm_ctrl` (brazo físico por CAN) + `tarea1_3_piramide`.

Ejemplos:
  # Simulación completa en casa:
  ros2 launch nero_ctrl tarea1_3_piramide.launch.py mode:=sim

  # Laboratorio (primero probar en el aire con +10 cm de margen):
  ros2 launch nero_ctrl tarea1_3_piramide.launch.py mode:=real can_port:=can0 z_offset_seguridad:=0.10
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    can_port = LaunchConfiguration("can_port").perform(context)
    z_mesa = float(LaunchConfiguration("z_mesa").perform(context))
    z_offset_seguridad = float(LaunchConfiguration("z_offset_seguridad").perform(context))
    velocidad_factor = float(LaunchConfiguration("velocidad_factor").perform(context))
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"

    ctrl_node = Node(
        package="nero_ctrl",
        executable="tarea1_3_piramide",
        name="tarea1_3_piramide",
        output="screen",
        parameters=[{
            "scene_name": "scene_piramide.xml",
            "z_mesa": z_mesa,
            "z_offset_seguridad": z_offset_seguridad,
            "velocidad_factor": velocidad_factor,
        }],
    )

    if mode == "real":
        backend = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("agx_arm_ctrl"),
                    "launch",
                    "start_single_agx_arm.launch.py",
                )
            ),
            launch_arguments={
                "can_port": can_port,
                "arm_type": "nero",
                "effector_type": "agx_gripper",
                "tcp_offset": "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
                "control_enabled": "true",
            }.items(),
        )
        return [backend, TimerAction(period=2.0, actions=[ctrl_node])]

    sim_args = ["--scene-name", "scene_piramide.xml", "--joint-speed", "1.3"]
    if headless:
        sim_args.append("--headless")

    sim_node = Node(
        package="nero_ctrl",
        executable="nero_sim_bridge",
        name="mujoco_bridge",
        output="screen",
        arguments=sim_args,
    )
    return [sim_node, TimerAction(period=1.5, actions=[ctrl_node])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="sim", choices=["sim", "real"]),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("z_mesa", default_value="0.064", description="Altura Z de la mesa (m)"),
        DeclareLaunchArgument("z_offset_seguridad", default_value="0.0", description="Margen vertical extra (m)"),
        DeclareLaunchArgument("velocidad_factor", default_value="1.0", description="Factor de velocidad (>1 más rápido, <1 más lento)"),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=launch_setup),
    ])

"""
tarea1_2_recoger_cinta.launch.py
================================
Lanza la Tarea 1.2 (recoger un cubo en movimiento sobre la cinta transportadora):
  - mode:=sim  (default): lanza `nero_sim_bridge` con `scene_cube.xml` + velocidad inicial
    de cinta (`cube_speed`, default 0.03 m/s) y el controlador `tarea1_2_recoger_cinta`.
  - mode:=real: lanza `agx_arm_ctrl` (brazo físico por CAN) y `tarea1_2_recoger_cinta`
    escuchando el sensor real (`/sensor/cube_distance` o `/cube/position`).

Ejemplos:
  # Simulación en casa:
  ros2 launch nero_ctrl tarea1_2_recoger_cinta.launch.py mode:=sim cube_speed:=0.03

  # Laboratorio (prueba segura 10 cm arriba antes de bajar a 0.0):
  ros2 launch nero_ctrl tarea1_2_recoger_cinta.launch.py mode:=real can_port:=can0 z_offset_seguridad:=0.10
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
    cube_speed = LaunchConfiguration("cube_speed").perform(context)
    distance_unit = LaunchConfiguration("distance_unit").perform(context)
    z_offset_seguridad = float(LaunchConfiguration("z_offset_seguridad").perform(context))
    depositar_y_repetir = LaunchConfiguration("depositar_y_repetir").perform(context).lower() == "true"
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"

    ctrl_node = Node(
        package="nero_ctrl",
        executable="tarea1_2_recoger_cinta",
        name="tarea1_2_recoger_cinta",
        output="screen",
        parameters=[{
            "scene_name": "scene_cube.xml",
            "distance_unit": distance_unit,
            "z_offset_seguridad": z_offset_seguridad,
            "stop_conveyor_on_grasp": (mode == "sim"),
            "depositar_y_repetir": depositar_y_repetir,
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

    sim_args = [
        "--scene-name", "scene_cube.xml",
        "--joint-speed", "1.2",
        "--init-ctrl", f"cube_motor={cube_speed}",
    ]
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
        DeclareLaunchArgument("cube_speed", default_value="0.03", description="Velocidad de la cinta en sim (m/s)"),
        DeclareLaunchArgument("distance_unit", default_value="m", choices=["m", "cm"]),
        DeclareLaunchArgument("z_offset_seguridad", default_value="0.0", description="Margen vertical extra (m) para pruebas seguras"),
        DeclareLaunchArgument("depositar_y_repetir", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=launch_setup),
    ])

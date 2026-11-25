"""
tarea2_2_evasion_movil.launch.py
================================
Lanza la Tarea 2.2 (brazo moviéndose a velocidad constante de extremo a extremo
de la cinta evitando cubos en movimiento) con la arquitectura de 3 nodos en una
única terminal:
  1. `nero_sim_bridge` (sim) o `agx_arm_ctrl` (real)
  2. `controller_node` (controlador reactivo puro de evasión)
  3. `tarea2_2_movil` (publica trayectoria ida y vuelta a velocidad constante en `/control/nominal_target`)

Ejemplos:
  # Simulación en casa:
  ros2 launch nero_ctrl tarea2_2_evasion_movil.launch.py mode:=sim cube_speed:=0.06

  # Ajustar velocidad de patrullaje del brazo:
  ros2 launch nero_ctrl tarea2_2_evasion_movil.launch.py mode:=sim cube_speed:=0.06 velocidad_y:=0.06

  # Laboratorio (probar primero elevado con margen de seguridad):
  ros2 launch nero_ctrl tarea2_2_evasion_movil.launch.py mode:=real can_port:=can0 z_offset_seguridad:=0.08
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
    y_min = float(LaunchConfiguration("y_min").perform(context))
    y_max = float(LaunchConfiguration("y_max").perform(context))
    velocidad_y = float(LaunchConfiguration("velocidad_y").perform(context))
    z_offset_seguridad = float(LaunchConfiguration("z_offset_seguridad").perform(context))
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"

    ctrl_node = Node(
        package="nero_ctrl",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[{
            "scene_name": "scene_evasion_cinta.xml",
            "distance_unit": distance_unit,
            "z_offset_seguridad": z_offset_seguridad,
        }],
    )

    task_node = Node(
        package="nero_ctrl",
        executable="tarea2_2_movil",
        name="tarea2_2_movil",
        output="screen",
        parameters=[{
            "y_min": y_min,
            "y_max": y_max,
            "velocidad_y": velocidad_y,
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
        return [backend, TimerAction(period=2.0, actions=[ctrl_node, task_node])]

    sim_args = [
        "--scene-name", "scene_evasion_cinta.xml",
        "--joint-speed", "2.0",
        "--loop-cubes",
        "--init-ctrl", f"cube1_motor={cube_speed}",
        "--init-ctrl", f"cube2_motor={cube_speed}",
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
    return [sim_node, TimerAction(period=1.5, actions=[ctrl_node, task_node])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="sim", choices=["sim", "real"]),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("cube_speed", default_value="0.06", description="Velocidad de cubos en sim (m/s)"),
        DeclareLaunchArgument("distance_unit", default_value="m", choices=["m", "cm"]),
        DeclareLaunchArgument("y_min", default_value="-0.20", description="Extremo inicial en Y (m)"),
        DeclareLaunchArgument("y_max", default_value="0.20", description="Extremo final en Y (m)"),
        DeclareLaunchArgument("velocidad_y", default_value="0.10", description="Velocidad lineal constante en Y (m/s)"),
        DeclareLaunchArgument("z_offset_seguridad", default_value="0.0", description="Margen vertical extra (m)"),
        DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=launch_setup),
    ])

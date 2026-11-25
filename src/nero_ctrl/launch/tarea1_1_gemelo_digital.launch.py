"""
tarea1_1_gemelo_digital.launch.py
=================================
Lanza el gemelo digital en MuJoCo para la Tarea 1.1:
  - mode:=real (default): lanza solo el visor pasivo `tarea1_1_gemelo`,
    escuchando /feedback/joint_states (del brazo real agx_arm_ctrl) y
    /sensor/cube_distance (del Arduino) o /cube/position.
  - mode:=demo: lanza además `tarea1_1_emulador` para probar visualmente
    desde casa sin brazo físico ni sensor.

Ejemplos:
  # En casa (prueba visual completa):
  ros2 launch nero_ctrl tarea1_1_gemelo_digital.launch.py mode:=demo

  # En el laboratorio (brazo real + Arduino publicando en cm):
  ros2 launch nero_ctrl tarea1_1_gemelo_digital.launch.py mode:=real distance_unit:=cm
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    scene_xml = LaunchConfiguration("scene_xml").perform(context)
    distance_unit = LaunchConfiguration("distance_unit").perform(context)
    sensor_y_origin = float(LaunchConfiguration("sensor_y_origin").perform(context))
    sensor_direction = float(LaunchConfiguration("sensor_direction").perform(context))
    cube_distance_topic = LaunchConfiguration("cube_distance_topic").perform(context)
    cube_point_topic = LaunchConfiguration("cube_point_topic").perform(context)
    cube_speed = float(LaunchConfiguration("cube_speed").perform(context))
    publish_mode = LaunchConfiguration("publish_mode").perform(context)

    actions = [
        Node(
            package="nero_ctrl",
            executable="tarea1_1_gemelo",
            name="gemelo_digital_mujoco",
            output="screen",
            parameters=[{
                "scene_xml": scene_xml,
                "distance_unit": distance_unit,
                "sensor_y_origin": sensor_y_origin,
                "sensor_direction": sensor_direction,
                "cube_distance_topic": cube_distance_topic,
                "cube_point_topic": cube_point_topic,
            }],
        )
    ]

    if mode == "demo":
        actions.append(
            Node(
                package="nero_ctrl",
                executable="tarea1_1_emulador",
                name="emulador_estado_real",
                output="screen",
                parameters=[{
                    "cube_speed": cube_speed,
                    "publish_mode": publish_mode,
                }],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="real",
            choices=["real", "demo"],
            description="real: escucha hardware real | demo: lanza emulador local de brazo y cubo",
        ),
        DeclareLaunchArgument(
            "scene_xml",
            default_value="",
            description="Ruta opcional a scene_cube.xml (vacio = autodeteccion)",
        ),
        DeclareLaunchArgument(
            "distance_unit",
            default_value="m",
            choices=["m", "cm"],
            description="Unidad de /sensor/cube_distance (m o cm)",
        ),
        DeclareLaunchArgument(
            "sensor_y_origin",
            default_value="-0.25",
            description="Coordenada Y (m) donde el sensor marca distancia 0",
        ),
        DeclareLaunchArgument(
            "sensor_direction",
            default_value="1.0",
            description="Sentido de avance en Y al aumentar la distancia (+1.0 o -1.0)",
        ),
        DeclareLaunchArgument(
            "cube_distance_topic",
            default_value="/sensor/cube_distance",
            description="Topico std_msgs/Float64 del sensor de distancia del cubo",
        ),
        DeclareLaunchArgument(
            "cube_point_topic",
            default_value="/cube/position",
            description="Topico geometry_msgs/PointStamped con posicion 3D del cubo",
        ),
        DeclareLaunchArgument(
            "cube_speed",
            default_value="0.05",
            description="Velocidad simulada del cubo en m/s (solo mode:=demo)",
        ),
        DeclareLaunchArgument(
            "publish_mode",
            default_value="distance",
            choices=["distance", "point"],
            description="Tipo de topico de cubo publicado por el emulador (solo mode:=demo)",
        ),
        OpaqueFunction(function=launch_setup),
    ])

"""Startet die Host-Seite des Roboters mit einem Befehl (Stufe 11h).

Kette:  shape_driver --/cmd_vel_raw--> obstacle_stop --/cmd_vel--> ESP32
        /range, /bumper --> obstacle_stop --/robot_state--> ESP32 (Display)
        /range --> distance_warner --/buzzer--> ESP32

Der micro-ROS-Agent wird bewusst NICHT mitgestartet (eigenes Terminal), damit die
Verbindung zum ESP32 bestehen bleibt, wenn die Nodes neu gestartet werden.

Argumente (Standardwert in Klammern):
  gate          (true)   obstacle_stop starten; einziger Publisher auf /cmd_vel.
                         false: shape_driver fährt DIREKT auf /cmd_vel – ohne Hindernisstopp und ohne Bumper!
  warner        (true)   distance_warner (Buzzer) starten
  figur         (false)  shape_driver starten (Viereck), 3 s verzögert, damit obstacle_stop schon Daten hat
  figur_params  (config/robot_imu.yaml)  Parameterdatei für shape_driver
  turn_mode     (imu)    imu oder time
  turn_lead_deg (4.0)    Vorhalt beim Drehen im IMU-Modus (Stufe 11f-c)

Beispiele:
  ros2 launch shape_driver robot_launch.py                              # nur Sicherheit + Warner (z. B. für teleop)
  ros2 launch shape_driver robot_launch.py figur:=true                  # Viereck mit IMU-Drehung
  ros2 launch shape_driver robot_launch.py figur:=true turn_mode:=time  # Viereck zeitgesteuert

Stand 09.10.2026: getestet ohne Fahrt (Test 1) und mit figur:=true, turn_mode imu (Test 2, Schreibtisch);
gate:=false und turn_mode:=time über die Launch-Datei UNGETESTET.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('shape_driver'), 'config')

    gate = LaunchConfiguration('gate')
    warner = LaunchConfiguration('warner')
    figur = LaunchConfiguration('figur')
    figur_params = LaunchConfiguration('figur_params')
    turn_mode = LaunchConfiguration('turn_mode')
    turn_lead = LaunchConfiguration('turn_lead_deg')

    args = [
        DeclareLaunchArgument('gate', default_value='true',
                              description='obstacle_stop zwischen Fahr-Node und /cmd_vel'),
        DeclareLaunchArgument('warner', default_value='true', description='distance_warner starten'),
        DeclareLaunchArgument('figur', default_value='false', description='shape_driver starten'),
        DeclareLaunchArgument('figur_params', default_value=os.path.join(config, 'robot_imu.yaml'),
                              description='Parameterdatei für shape_driver'),
        DeclareLaunchArgument('turn_mode', default_value='imu', description='imu oder time'),
        DeclareLaunchArgument('turn_lead_deg', default_value='4.0', description='Vorhalt [°]'),
    ]

    obstacle_stop = Node(
        package='shape_driver', executable='obstacle_stop', name='obstacle_stop',
        parameters=[os.path.join(config, 'obstacle_stop.yaml')],
        output='screen', condition=IfCondition(gate))

    distance_warner = Node(
        package='shape_driver', executable='distance_warner', name='distance_warner',
        parameters=[os.path.join(config, 'distance_warner.yaml')],
        output='screen', condition=IfCondition(warner))

    # Mit Sicherheitsfilter über /cmd_vel_raw, sonst direkt auf /cmd_vel
    cmd_topic = PythonExpression(["'/cmd_vel_raw' if '", gate, "'.lower() == 'true' else '/cmd_vel'"])
    shape_driver = Node(
        package='shape_driver', executable='shape_driver_node', name='shape_driver',
        parameters=[figur_params, {
            'cmd_vel_topic': cmd_topic,
            'turn_mode': ParameterValue(turn_mode, value_type=str),
            'turn_lead_deg': ParameterValue(turn_lead, value_type=float),
        }],
        output='screen')

    return LaunchDescription(args + [
        LogInfo(msg='Agent separat starten: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888'),
        LogInfo(msg='WARNUNG: gate:=false – kein Hindernisstopp, kein Bumper-Stopp!',
                condition=UnlessCondition(gate)),
        obstacle_stop,
        distance_warner,
        TimerAction(period=3.0, actions=[shape_driver], condition=IfCondition(figur)),
    ])

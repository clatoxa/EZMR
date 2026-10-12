"""Erkundungsfahrt mit Ultraschallkarte (Baustein 4) – startet die gesamte Host-Seite.

  wander --/cmd_vel_raw--> obstacle_stop --/cmd_vel--> ESP32
  ESP32 --/wheel_cmd, /imu--> odom --/odom, TF odom->base_link, /odom_path-->
  ESP32 --/range--> sonar_map (+ /odom) --/map--> RViz2 ; beim Beenden Karte speichern

Der micro-ROS-Agent wird NICHT mitgestartet (eigenes Terminal, wie robot_launch.py).

Ablauf: alle Nodes starten sofort; wander startet erst nach wander_delay Sekunden, damit odom
im Stillstand die Gyroskop-Drift lernt (Stufe 11i). Nach duration Sekunden hält wander an;
odom und sonar_map laufen weiter. Strg+C beendet alles, sonar_map speichert dabei die Karte
nach ~/messungen/karten/.

Argumente (Standard):
  duration      (300)   s Zufallsfahrt
  wander_delay  (20)    s Wartezeit vor der Fahrt
  sensor_y      (0.0)   m seitlicher Versatz des Ultraschallsensors (+ links, − rechts); Sensor mittig
                        (Sender/Empfänger je 1,5 cm links/rechts der Mitte, Messung 12.10.)
  warner        (true)  Buzzer
  rviz          (true)  RViz2 mit config/explore.rviz
  seed          (-1)    Zufallsfolge (>= 0: reproduzierbar)
  imu_reset     (true)  beim Start /imu_reset senden: ESP32 misst Gyro-Nullpunkt neu, Winkel 0 (Firmware ≥ v7.3)
  fwd_min / fwd_max (6 / 15) s: nach zufälliger Fahrzeit freiwillig abbiegen (gegen Pendeln, Test E3)
  turn_min_deg / turn_max_deg (90 / 90)  Drehwinkel beim Ausweichen; 90/90 = immer ±90° (Richtung
                        zufällig) -> rechtwinklige Bahn; bei Start 45° zu den Dielenfugen bleiben alle
                        Fahrtrichtungen schräg zu den Fugen (weniger Bodenecho, Stufe 11g)

Stand 09.10.2026: getestet (E1–E3); imu_reset und fwd_min/fwd_max UNGETESTET.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('shape_driver'), 'config')
    lc = LaunchConfiguration

    args = [
        DeclareLaunchArgument('duration', default_value='300.0'),
        DeclareLaunchArgument('wander_delay', default_value='20.0'),
        DeclareLaunchArgument('sensor_y', default_value='0.0'),
        DeclareLaunchArgument('warner', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('imu_reset', default_value='true'),
        DeclareLaunchArgument('fwd_min', default_value='6.0'),
        DeclareLaunchArgument('fwd_max', default_value='15.0'),
        DeclareLaunchArgument('turn_min_deg', default_value='90.0'),
        DeclareLaunchArgument('turn_max_deg', default_value='90.0'),
    ]

    nodes = [
        Node(package='shape_driver', executable='obstacle_stop', name='obstacle_stop',
             parameters=[os.path.join(config, 'obstacle_stop.yaml')], output='screen'),
        Node(package='shape_driver', executable='distance_warner', name='distance_warner',
             parameters=[os.path.join(config, 'distance_warner.yaml')], output='screen',
             condition=IfCondition(lc('warner'))),
        Node(package='shape_driver', executable='odom', name='odom', output='screen'),
        Node(package='shape_driver', executable='sonar_map', name='sonar_map', output='screen',
             parameters=[{'sensor_y': ParameterValue(lc('sensor_y'), value_type=float)}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', os.path.join(config, 'explore.rviz')],
             condition=IfCondition(lc('rviz'))),
    ]

    wander = Node(
        package='shape_driver', executable='wander', name='wander', output='screen',
        parameters=[{
            'duration': ParameterValue(lc('duration'), value_type=float),
            'seed': ParameterValue(lc('seed'), value_type=int),
            'turn_min_deg': ParameterValue(lc('turn_min_deg'), value_type=float),
            'turn_max_deg': ParameterValue(lc('turn_max_deg'), value_type=float),
            'max_forward_min': ParameterValue(lc('fwd_min'), value_type=float),
            'max_forward_max': ParameterValue(lc('fwd_max'), value_type=float),
        }])

    # Firmware v7.3: Nullpunkt neu messen + Winkel 0. Dreimal im Abstand von 1 s senden, falls die erste
    # Nachricht vor dem Verbindungsaufbau verloren geht; der ESP32 ignoriert Anfragen während der Messung.
    imu_reset = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--times', '3', '--rate', '1', '/imu_reset',
             'std_msgs/msg/UInt8', '{data: 1}'],
        output='log', condition=IfCondition(lc('imu_reset')))

    return LaunchDescription(args + [
        LogInfo(msg='Agent separat starten. Roboter jetzt NICHT bewegen – Driftschätzung läuft.'),
        *nodes,
        imu_reset,
        TimerAction(period=lc('wander_delay'), actions=[
            LogInfo(msg='Zufallsfahrt startet.'),
            wander,
        ]),
    ])

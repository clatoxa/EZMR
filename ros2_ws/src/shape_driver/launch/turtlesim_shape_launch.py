"""Startet turtlesim und die Fahr-Node shape_driver mit Parametern aus einer YAML-Datei."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('shape_driver'), 'config', 'turtlesim.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML-Datei mit den Fahrparametern'),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'),

        # Fahr-Node verzögert starten, damit turtlesim bereit ist und
        # die ersten Bewegungsbefehle nicht verloren gehen.
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='shape_driver',
                    executable='shape_driver_node',
                    name='shape_driver',
                    parameters=[params_file],
                    output='screen'),
            ]),
    ])

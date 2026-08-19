#!/usr/bin/env python3
"""
TurtleBot 4 - Formfahrt Node (Dauerschleife)
Verwendung:
  ros2 run <dein_paket> turtlebot_shape --ros-args -p shape:=square
  ros2 run <dein_paket> turtlebot_shape --ros-args -p shape:=eight
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class ShapeDriver(Node):
    def __init__(self):
        super().__init__('shape_driver')

        self.declare_parameter('shape', 'square')
        self.shape = self.get_parameter('shape').get_parameter_value().string_value

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        time.sleep(1.0)

        self.get_logger().info(f'Starte Formfahrt in Dauerschleife: {self.shape}')
        self.get_logger().info('Mit Strg+C stoppen.')

        try:
            while rclpy.ok():
                if self.shape == 'square':
                    self.drive_square()
                elif self.shape == 'eight':
                    self.drive_eight()
                else:
                    self.get_logger().warn(f'Unbekannte Form: {self.shape}. Nutze square oder eight.')
                    self.drive_square()
        except KeyboardInterrupt:
            self.get_logger().info('Gestoppt.')
        finally:
            self.stop()

    def publish_velocity(self, linear: float, angular: float, duration: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher.publish(msg)
            time.sleep(0.1)

    def stop(self):
        msg = Twist()
        self.publisher.publish(msg)
        time.sleep(0.5)

    def drive_square(self, side_length_time=3.0, turn_time=1.57):
        self.get_logger().info('Fahre Viereck...')
        for i in range(4):
            self.get_logger().info(f'  Seite {i+1}/4: Geradeaus')
            self.publish_velocity(linear=0.2, angular=0.0, duration=side_length_time)
            self.stop()
            self.get_logger().info(f'  Ecke {i+1}/4: Drehen 90°')
            self.publish_velocity(linear=0.0, angular=1.0, duration=turn_time)
            self.stop()

    def drive_eight(self, circle_time=6.28):
        self.get_logger().info('Fahre Acht...')
        self.get_logger().info('  Erster Kreis (links)')
        self.publish_velocity(linear=0.2, angular=0.5, duration=circle_time)
        self.stop()
        self.get_logger().info('  Zweiter Kreis (rechts)')
        self.publish_velocity(linear=0.2, angular=-0.5, duration=circle_time)
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDriver()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Fahrt eine regelmäßige Vielecksfigur (z. B. Viereck) über geometry_msgs/Twist.

Zeitgesteuert (ohne Positionsrückführung). Alle Fahrwerte sind ROS-Parameter,
damit dieselbe Node später auch den realen Roboter ansteuern kann.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ShapeDriver(Node):
    FORWARD = 'FORWARD'
    TURN = 'TURN'
    DONE = 'DONE'

    def __init__(self):
        super().__init__('shape_driver')

        # Parameter mit Standardwerten für turtlesim
        self.declare_parameter('cmd_vel_topic', '/turtle1/cmd_vel')
        self.declare_parameter('num_sides', 4)
        self.declare_parameter('side_length', 2.0)    # Einheiten der Zielumgebung
        self.declare_parameter('linear_speed', 1.0)   # Einheiten pro Sekunde
        self.declare_parameter('angular_speed', 0.5)  # rad/s
        self.declare_parameter('publish_rate', 10.0)  # Hz

        topic = self.get_parameter('cmd_vel_topic').value
        self.num_sides = int(self.get_parameter('num_sides').value)
        side_length = float(self.get_parameter('side_length').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.finished = False
        if (self.num_sides < 3 or side_length <= 0.0 or self.linear_speed <= 0.0
                or self.angular_speed <= 0.0 or rate <= 0.0):
            self.get_logger().error('Ungültige Parameter – Node wird beendet.')
            self.finished = True
            return

        # Phasendauern aus der Geometrie berechnen
        self.forward_duration = side_length / self.linear_speed
        self.turn_duration = (2.0 * math.pi / self.num_sides) / self.angular_speed

        self.publisher = self.create_publisher(Twist, topic, 10)

        self.sides_done = 0
        self.done_ticks = 0
        self.state = self.FORWARD
        self.state_start = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f'Start: {self.num_sides} Seiten, Topic {topic}, '
            f'Fahrzeit {self.forward_duration:.2f} s, Drehzeit {self.turn_duration:.2f} s')

    def elapsed(self):
        """Sekunden seit Beginn des aktuellen Zustands."""
        return (self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def switch_to(self, new_state):
        self.state = new_state
        self.state_start = self.get_clock().now()
        self.get_logger().info(f'Zustand -> {new_state} (fertige Seiten: {self.sides_done})')

    def update_state(self):
        t = self.elapsed()
        if self.state == self.FORWARD and t >= self.forward_duration:
            self.switch_to(self.TURN)
        elif self.state == self.TURN and t >= self.turn_duration:
            self.sides_done += 1
            if self.sides_done >= self.num_sides:
                self.switch_to(self.DONE)
            else:
                self.switch_to(self.FORWARD)

    def on_timer(self):
        self.update_state()

        msg = Twist()  # alle Felder sind standardmäßig 0.0
        if self.state == self.FORWARD:
            msg.linear.x = self.linear_speed
        elif self.state == self.TURN:
            msg.angular.z = self.angular_speed
        self.publisher.publish(msg)

        # Nach Abschluss einige Nullbefehle senden, dann beenden
        if self.state == self.DONE:
            self.done_ticks += 1
            if self.done_ticks >= 5:
                self.timer.cancel()
                self.get_logger().info('Fahrfolge abgeschlossen.')
                self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDriver()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

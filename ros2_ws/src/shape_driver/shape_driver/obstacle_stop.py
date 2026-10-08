"""Sicherheitsfilter (Stufe 10b): leitet Fahrbefehle weiter, sperrt Vorwärtsfahrt bei Hindernis.

    Fahr-Node --> /cmd_vel_raw --> [obstacle_stop] --> /cmd_vel --> ESP32
                                         ^
                       /range -----------+

Diese Node ist der EINZIGE Publisher auf /cmd_vel. Fahr-Nodes (shape_driver,
calib_drive, teleop) werden per Parameter auf /cmd_vel_raw umgeleitet.

Verhalten:
- Befehle werden unverändert weitergegeben, nur linear.x > 0 wird auf 0 gesetzt,
  solange "gesperrt" ist. Drehen und Rückwärtsfahren bleiben erlaubt.
- Medianfilter über die letzten Messwerte gegen einzelne Ausreißer.
- Hysterese: sperren unter stop_distance, freigeben erst über release_distance.
- Fail-safe: ohne aktuelle /range-Werte (Start, Ausfall) ist vorwärts gesperrt.
- Sonderwerte (REP 117, Firmware v3): +inf = frei, -inf = zu nah,
  nan = kein Echo (Sensorfehler) -> beide wie Abstand 0 behandelt (sperren).

Stand 08.10.2026: UNGETESTET.
"""
import math
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class ObstacleStop(Node):

    def __init__(self):
        super().__init__('obstacle_stop')

        self.declare_parameter('input_topic', '/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('range_topic', '/range')
        self.declare_parameter('stop_distance', 0.20)     # m: darunter sperren
        self.declare_parameter('release_distance', 0.25)  # m: darüber freigeben
        self.declare_parameter('range_timeout', 0.5)      # s: ohne Messwert -> sperren
        self.declare_parameter('median_window', 3)        # Anzahl Werte im Medianfilter

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        range_topic = self.get_parameter('range_topic').value
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.release_distance = float(self.get_parameter('release_distance').value)
        self.range_timeout = float(self.get_parameter('range_timeout').value)
        window = int(self.get_parameter('median_window').value)

        if self.release_distance <= self.stop_distance or window < 1:
            raise ValueError('release_distance muss > stop_distance sein, median_window >= 1')

        self.window = deque(maxlen=window)
        self.blocked = True              # fail-safe: bis zum ersten Messwert gesperrt
        self.last_range_time = None      # Empfangszeit (Host-Uhr) des letzten Messwerts
        self.last_median = float('nan')

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self.on_cmd, 10)
        self.create_subscription(Range, range_topic, self.on_range, 10)

        self.get_logger().info(
            f'{in_topic} -> {out_topic}, Sensor {range_topic}, '
            f'Stopp < {self.stop_distance:.2f} m, Freigabe > {self.release_distance:.2f} m, '
            f'Timeout {self.range_timeout:.2f} s, Median über {window}')

    def on_range(self, msg):
        r = msg.range
        if math.isnan(r) or r < 0.0:     # nan = kein Echo, -inf = zu nah
            r = 0.0
        self.window.append(r)
        self.last_median = statistics.median(self.window)
        self.last_range_time = self.get_clock().now()

        # Hysterese
        if self.blocked and self.last_median > self.release_distance:
            self.blocked = False
            self.get_logger().info(f'FREI (Abstand {self.last_median:.3f} m)')
        elif not self.blocked and self.last_median < self.stop_distance:
            self.blocked = True
            self.get_logger().warn(f'HINDERNIS -> vorwärts gesperrt (Abstand {self.last_median:.3f} m)')

    def range_is_stale(self):
        if self.last_range_time is None:
            return True
        age = (self.get_clock().now() - self.last_range_time).nanoseconds / 1e9
        return age > self.range_timeout

    def on_cmd(self, msg):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z

        stale = self.range_is_stale()
        if stale:
            self.get_logger().warn('Keine aktuellen /range-Werte -> vorwärts gesperrt',
                                   throttle_duration_sec=2.0)
        if (self.blocked or stale) and out.linear.x > 0.0:
            out.linear.x = 0.0
        self.pub.publish(out)

    def send_stop(self):
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Humble fährt bei Strg+C den ROS-Kontext selbst herunter; danach ist
        # publish() nicht mehr möglich. In dem Fall stoppt der Firmware-Timeout (500 ms).
        if rclpy.ok():
            node.send_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

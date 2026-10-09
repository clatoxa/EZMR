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

Stufe 11: veröffentlicht zusätzlich den Zustand auf /robot_state (std_msgs/UInt8) für das Display:
1 = FREI, 2 = HINDERNIS (vorwärts gesperrt), 3 = keine aktuellen Sensordaten (vorwärts gesperrt).
Senden bei Zustandswechsel sofort und sonst alle 0,2 s.

Stufe 11/12 (v6): wertet zusätzlich /bumper (std_msgs/UInt8, Bit 0 links, Bit 1 rechts) aus.
Bumper gedrückt -> vorwärts gesperrt, /robot_state = 4 (BUMPER). Freigabe, sobald kein Bumper mehr gedrückt ist.
Fail-safe: mit use_bumper = true sperren auch fehlende /bumper-Daten (> bumper_timeout).

Stand 09.10.2026: Grundfunktion getestet (10b), /robot_state getestet (11c); Bumper-Auswertung UNGETESTET.
"""
import math
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import UInt8

STATE_FREE = 1
STATE_OBSTACLE = 2
STATE_NO_DATA = 3
STATE_BUMPER = 4


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
        self.declare_parameter('state_topic', '/robot_state')
        self.declare_parameter('bumper_topic', '/bumper')
        self.declare_parameter('use_bumper', True)
        self.declare_parameter('bumper_timeout', 1.0)   # s

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
        self.state_pub = self.create_publisher(UInt8, self.get_parameter('state_topic').value, 10)
        self.use_bumper = bool(self.get_parameter('use_bumper').value)
        self.bumper_timeout = float(self.get_parameter('bumper_timeout').value)
        self.bumper = 0
        self.last_bumper_time = None
        if self.use_bumper:
            self.create_subscription(UInt8, self.get_parameter('bumper_topic').value, self.on_bumper, 10)
        self.last_state = None
        self.create_timer(0.2, self.publish_state)

        self.get_logger().info(
            f'{in_topic} -> {out_topic}, Sensor {range_topic}, '
            f'Stopp < {self.stop_distance:.2f} m, Freigabe > {self.release_distance:.2f} m, '
            f'Timeout {self.range_timeout:.2f} s, Median über {window}, '
            f'Bumper {"an" if self.use_bumper else "aus"}')

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
        if self.current_state() != self.last_state:
            self.publish_state()                 # Zustandswechsel sofort melden (Display-Ereignis)

    def on_bumper(self, msg):
        if msg.data != self.bumper:
            if msg.data:
                seite = {1: 'LINKS', 2: 'RECHTS', 3: 'BEIDE'}.get(msg.data, str(msg.data))
                self.get_logger().warn(f'BUMPER {seite} -> vorwärts gesperrt')
            else:
                self.get_logger().info('Bumper frei')
        self.bumper = msg.data
        self.last_bumper_time = self.get_clock().now()
        if self.current_state() != self.last_state:
            self.publish_state()

    def bumper_is_stale(self):
        if not self.use_bumper:
            return False
        if self.last_bumper_time is None:
            return True
        age = (self.get_clock().now() - self.last_bumper_time).nanoseconds / 1e9
        return age > self.bumper_timeout

    def current_state(self):
        if self.range_is_stale() or self.bumper_is_stale():
            return STATE_NO_DATA
        if self.use_bumper and self.bumper != 0:
            return STATE_BUMPER
        return STATE_OBSTACLE if self.blocked else STATE_FREE

    def publish_state(self):
        msg = UInt8()
        msg.data = self.current_state()
        self.state_pub.publish(msg)
        self.last_state = msg.data

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
        bumper_stale = self.bumper_is_stale()
        if bumper_stale:
            self.get_logger().warn('Keine aktuellen /bumper-Werte -> vorwärts gesperrt',
                                   throttle_duration_sec=2.0)
        bumper_pressed = self.use_bumper and self.bumper != 0
        if (self.blocked or stale or bumper_stale or bumper_pressed) and out.linear.x > 0.0:
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

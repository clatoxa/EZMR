"""Kalibrierfahrt: sendet nach einer Startverzögerung für eine feste Dauer einen konstanten
Twist und danach mehrere Nullbefehle (sauberer Stopp, unabhängig vom Timeout des ESP32).

Beispiele:
  1 m geradeaus:  linear_speed:=0.031447 duration:=31.80
  360° drehen:    angular_speed:=0.381171 duration:=16.48
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CalibDrive(Node):

    def __init__(self):
        super().__init__('calib_drive')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.0)   # m/s
        self.declare_parameter('angular_speed', 0.0)  # rad/s
        self.declare_parameter('duration', 0.0)       # s
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('start_delay', 3.0)    # s, Zeit zum Zurücktreten

        topic = self.get_parameter('cmd_vel_topic').value
        self.v = float(self.get_parameter('linear_speed').value)
        self.w = float(self.get_parameter('angular_speed').value)
        duration = float(self.get_parameter('duration').value)
        rate = float(self.get_parameter('publish_rate').value)
        delay = float(self.get_parameter('start_delay').value)

        self.finished = False
        if duration <= 0.0 or rate <= 0.0 or delay < 0.0:
            self.get_logger().error('Ungültige Parameter (duration > 0, publish_rate > 0, start_delay >= 0)')
            self.finished = True
            return

        self.publisher = self.create_publisher(Twist, topic, 10)
        now_s = self.now_s()
        self.t_start = now_s + delay
        self.t_end = self.t_start + duration
        self.started = False
        self.stop_ticks = 0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f'Kalibrierfahrt auf {topic}: v = {self.v:.6f} m/s, w = {self.w:.6f} rad/s, '
            f'Dauer {duration:.2f} s → Soll: {self.v * duration:.3f} m, '
            f'{math.degrees(self.w * duration):.1f}°. Start in {delay:.1f} s.')

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_timer(self):
        t = self.now_s()
        msg = Twist()  # Nullbefehl als Standard
        if self.t_start <= t < self.t_end:
            if not self.started:
                self.get_logger().info('Fahrt gestartet')
                self.started = True
            msg.linear.x = self.v
            msg.angular.z = self.w
        elif t >= self.t_end:
            self.stop_ticks += 1
            if self.stop_ticks == 1:
                self.get_logger().info(
                    f'Fahrt beendet (Ist-Dauer laut Uhr: {t - self.t_start:.2f} s), sende Stopp')
        self.publisher.publish(msg)
        if self.stop_ticks >= 5:
            self.timer.cancel()
            self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = CalibDrive()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Bei Abbruch (Strg+C) zur Sicherheit einen Nullbefehl senden
        if hasattr(node, 'publisher') and rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

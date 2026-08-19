#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time
import signal
import sys

class CirclePublisher(Node):
    def __init__(self):
        super().__init__('circle_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.01, self.publish_circle)
        self.start_time = time.time()
        self.radius = 0.5    # Meter
        self.v = 1.0         # m/s = Bahngeschwindigkeit
        self.omega = self.v / self.radius  # Kreisfrequenz

    def publish_circle(self):
        t = time.time() - self.start_time
        msg = Twist()
        msg.linear.x = self.v * math.cos(self.omega * t)
        msg.linear.y = self.v * math.sin(self.omega * t)
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def publish_stop(self):
        msg = Twist()  # Alle Werte auf 0
        self.publisher.publish(msg)
        print("Stop-Message published.")

def main(args=None):
    rclpy.init(args=args)
    node = CirclePublisher()

    # Signal-Handler für Ctrl+C (SIGINT)
    def signal_handler(sig, frame):
        print("\nAbbruch erkannt: Stoppe den Roboter ...")
        node.publish_stop()
        time.sleep(0.1)  # Kurz warten, damit die Nachricht rausgeht
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Wird schon vom Signal-Handler behandelt

if __name__ == '__main__':
    main()

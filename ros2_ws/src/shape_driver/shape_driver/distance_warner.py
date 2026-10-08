"""Stufe 10d: akustische Abstandswarnung (wie eine Einparkhilfe).

    /range (sensor_msgs/Range) --> [distance_warner] --> /buzzer (std_msgs/UInt16) --> ESP32

Die Auswertung (Filter, Stufen) läuft auf dem Host, der ESP32 erzeugt nur den Ton.
Bedeutung von /buzzer (Firmware v4): 0 = aus, 1 = Dauerton, >= 2 = Pieptakt in ms.

Stufen (Parameter, Vorschlagswerte): je kleiner der Abstand, desto schneller der Pieptakt.
Ohne aktuelle Messwerte wird "aus" gesendet (die Warnung ist eine Zusatzinformation;
die Sicherheitsfunktion bleibt bei obstacle_stop).
Sendet bei jedem /range-Wert (10 Hz), damit der Timeout im ESP32 (1 s) nicht auslöst.

Stand 08.10.2026: UNGETESTET.
"""
import math
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import UInt16


class DistanceWarner(Node):

    def __init__(self):
        super().__init__('distance_warner')

        self.declare_parameter('range_topic', '/range')
        self.declare_parameter('buzzer_topic', '/buzzer')
        self.declare_parameter('enabled', True)
        # Abstandsgrenzen (m, absteigend) und zugehöriger Pieptakt (ms; 1 = Dauerton)
        self.declare_parameter('distances', [0.50, 0.35, 0.20, 0.10])
        self.declare_parameter('periods_ms', [800, 400, 200, 1])
        self.declare_parameter('median_window', 3)
        self.declare_parameter('range_timeout', 0.5)   # s

        self.enabled = bool(self.get_parameter('enabled').value)
        self.distances = [float(d) for d in self.get_parameter('distances').value]
        self.periods = [int(p) for p in self.get_parameter('periods_ms').value]
        window = int(self.get_parameter('median_window').value)
        self.range_timeout = float(self.get_parameter('range_timeout').value)

        if (len(self.distances) != len(self.periods) or window < 1
                or any(a <= b for a, b in zip(self.distances, self.distances[1:]))
                or any(p < 1 or p > 65535 for p in self.periods)):
            raise ValueError('distances absteigend, gleich lang wie periods_ms (1..65535), median_window >= 1')

        self.window = deque(maxlen=window)
        self.last_range_time = None
        self.last_period = None

        self.pub = self.create_publisher(UInt16, self.get_parameter('buzzer_topic').value, 10)
        self.create_subscription(Range, self.get_parameter('range_topic').value, self.on_range, 10)
        # Überwachung: bleiben Messwerte aus, Buzzer ausschalten
        self.create_timer(0.2, self.on_watchdog)

        stufen = ', '.join(f'< {d:.2f} m: {"Dauerton" if p == 1 else f"{p} ms"}'
                           for d, p in zip(self.distances, self.periods))
        self.get_logger().info(f'Abstandswarnung {"AN" if self.enabled else "AUS"} – {stufen}')

    def period_for(self, d):
        """Pieptakt für Abstand d (0 = aus). Kleinste zutreffende Grenze gewinnt."""
        period = 0
        for limit, p in zip(self.distances, self.periods):
            if d < limit:
                period = p
        return period

    def send(self, period):
        msg = UInt16()
        msg.data = period
        self.pub.publish(msg)
        if period != self.last_period:
            text = 'aus' if period == 0 else ('Dauerton' if period == 1 else f'Takt {period} ms')
            self.get_logger().info(f'Buzzer: {text}')
            self.last_period = period

    def on_range(self, msg):
        r = msg.range
        if math.isnan(r) or r < 0.0:     # nan = kein Echo, -inf = zu nah -> wie sehr nah
            r = 0.0
        self.window.append(r)
        self.last_range_time = self.get_clock().now()
        median = statistics.median(self.window)
        self.send(self.period_for(median) if self.enabled else 0)

    def on_watchdog(self):
        if self.last_range_time is None:
            return
        age = (self.get_clock().now() - self.last_range_time).nanoseconds / 1e9
        if age > self.range_timeout and self.last_period != 0:
            self.get_logger().warn('Keine aktuellen /range-Werte -> Buzzer aus')
            self.send(0)


def main(args=None):
    rclpy.init(args=args)
    node = DistanceWarner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():                   # bei Strg+C ist der Kontext schon beendet (siehe 10b)
            node.send(0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

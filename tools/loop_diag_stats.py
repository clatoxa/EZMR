#!/usr/bin/env python3
"""Stufe 11d: /loop_diag eine feste Zeit lang aufzeichnen und Kennwerte ausgeben.

Aufruf (ROS-Umgebung gesourct):
    python3 loop_diag_stats.py [--dauer 60] [--name Variante]

/loop_diag (geometry_msgs/Vector3, 1 Wert pro Sekunde, Firmware v5):
    x = längster loop()-Durchlauf in dieser Sekunde [us]
    y = größter Abstand zwischen zwei Regelschritten [us] (Soll 50 000)
    z = kleinster Abstand zwischen zwei Regelschritten [us]

Stand 08.10.2026: UNGETESTET.
"""
import argparse
import statistics

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


def kennwerte(werte):
    w = sorted(werte)
    p90 = w[min(len(w) - 1, int(round(0.9 * (len(w) - 1))))]
    return (f'Median {statistics.median(w) / 1000:6.1f} ms | 90 % {p90 / 1000:6.1f} ms | '
            f'max {w[-1] / 1000:6.1f} ms | min {w[0] / 1000:6.1f} ms')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--dauer', type=float, default=60.0, help='Aufzeichnungsdauer in s')
    p.add_argument('--name', default='', help='Bezeichnung der Variante (nur für die Ausgabe)')
    a = p.parse_args()

    rclpy.init()
    node = Node('loop_diag_stats')
    daten = []
    node.create_subscription(Vector3, '/loop_diag', lambda m: daten.append((m.x, m.y, m.z)), 10)
    start = node.get_clock().now()
    print(f'Zeichne /loop_diag {a.dauer:.0f} s auf ...')
    try:
        while rclpy.ok() and (node.get_clock().now() - start).nanoseconds / 1e9 < a.dauer:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if len(daten) < 3:
        print(f'Zu wenige Werte ({len(daten)}). Läuft der ESP32 mit Firmware v5?')
        return
    # erster Wert enthält ggf. den Start -> verwerfen
    daten = daten[1:]
    x = [d[0] for d in daten]
    y = [d[1] for d in daten]
    z = [d[2] for d in daten]
    ueber = sum(1 for v in y if v > 55000)
    print(f'\n=== {a.name or "Variante"}: n = {len(daten)} Sekundenwerte ===')
    print(f'loop()-Durchlauf max/s : {kennwerte(x)}')
    print(f'Regelabstand max/s     : {kennwerte(y)}')
    print(f'Regelabstand min/s     : {kennwerte(z)}')
    print(f'Sekunden mit Regelabstand > 55 ms: {ueber} von {len(y)}')


if __name__ == '__main__':
    main()

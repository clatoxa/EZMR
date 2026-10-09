#!/usr/bin/env python3
"""Diagnose Scheinhindernisse: Roboter dreht auf der Stelle, /range wird über dem Gierwinkel erfasst.

Der Roboter dreht sich (nur angular.z, keine Vorwärtsfahrt) eine oder mehrere volle Umdrehungen.
Zu jedem /range-Wert wird der aktuelle Gierwinkel aus /imu gespeichert. Ergebnis:
Tabelle je Winkelsektor (Median, Min, Max, Anzahl nan/inf) und optional eine CSV-Datei.

Winkel relativ zur Startausrichtung, gegen den Uhrzeigersinn positiv (REP 103):
steht der Roboter wie beim Viereck am Start, entspricht 0° Seite 1, 90° Seite 2,
180° Seite 3, 270° Seite 4.

Aufruf (ROS-Umgebung gesourct, Agent und obstacle_stop laufen):
    python3 range_scan.py --name Startpunkt --csv scan_start.csv

Standardmäßig wird auf /cmd_vel_raw veröffentlicht (obstacle_stop lässt Drehen immer durch,
bleibt einziger Publisher auf /cmd_vel). Abbruch jederzeit mit Strg+C (Stoppbefehl wird gesendet).

Stand 09.10.2026: UNGETESTET.
"""
import argparse
import csv
import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class RangeScan(Node):
    def __init__(self, a):
        super().__init__('range_scan')
        self.a = a
        self.pub = self.create_publisher(Twist, a.topic, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Range, '/range', self.on_range, qos_profile_sensor_data)
        self.yaw = None            # fortlaufend (ausgerollt) [rad]
        self.yaw_raw_last = None
        self.imu_time = None
        self.yaw0 = None           # Startwinkel
        self.samples = []          # (t [s], Winkel relativ [°], range [m])
        self.t0 = None
        self.running = False
        self.done = False
        self.result = ''

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_imu(self, msg):
        y = yaw_from_quat(msg.orientation)
        if self.yaw_raw_last is None:
            self.yaw = y
        else:
            self.yaw += wrap(y - self.yaw_raw_last)
        self.yaw_raw_last = y
        self.imu_time = self.now_s()

    def on_range(self, msg):
        if self.running and self.yaw is not None:
            self.samples.append((self.now_s() - self.t0, math.degrees(self.yaw - self.yaw0), msg.range))

    def imu_ok(self):
        return self.imu_time is not None and self.now_s() - self.imu_time <= 0.5

    def step(self):
        """Ein Regelschritt (20 Hz). Gibt False zurück, wenn fertig."""
        if not self.running:
            if self.imu_ok():
                self.yaw0 = self.yaw
                self.t0 = self.now_s()
                self.running = True
                self.get_logger().info(f'Start: {self.a.umdrehungen} Umdrehung(en) mit {self.a.omega} rad/s')
            elif self.t0 is None and self.now_s() - self.start_wait > 5.0:
                self.result = 'Keine /imu-Daten innerhalb 5 s – läuft Firmware v7.1?'
                return False
            return True
        if not self.imu_ok():
            self.result = 'Abbruch: /imu-Daten ausgeblieben'
            return False
        turned = self.yaw - self.yaw0
        soll = 2.0 * math.pi * self.a.umdrehungen
        if turned >= soll:
            self.result = f'Fertig: {math.degrees(turned):.1f}° gedreht'
            return False
        if self.now_s() - self.t0 > 2.0 * soll / self.a.omega + 5.0:
            self.result = 'Abbruch: Drehung dauert zu lange (dreht der Roboter?)'
            return False
        msg = Twist()
        msg.angular.z = self.a.omega
        self.pub.publish(msg)
        return True

    def stop(self):
        for _ in range(5):
            self.pub.publish(Twist())


def auswerten(samples, sektor, name):
    print(f'\n=== {name or "Scan"}: {len(samples)} /range-Werte ===')
    print(' Sektor [°] |   n | Median [m] | Min [m] | Max [m] | nan | inf | -inf')
    n_sek = int(round(360 / sektor))
    for i in range(n_sek):
        lo = i * sektor
        werte = [r for _, w, r in samples if lo <= (w % 360.0) < lo + sektor]
        if not werte:
            print(f' {lo:4d}–{lo + sektor:<4d} |   0 |')
            continue
        endl = [r for r in werte if math.isfinite(r)]
        n_nan = sum(1 for r in werte if math.isnan(r))
        n_inf = sum(1 for r in werte if math.isinf(r) and r > 0)
        n_ninf = sum(1 for r in werte if math.isinf(r) and r < 0)
        if endl:
            med = statistics.median(endl)
            mark = '  ◄ < 0,20 m' if med < 0.20 else ''
            print(f' {lo:4d}–{lo + sektor:<4d} | {len(werte):3d} | {med:10.3f} | {min(endl):7.3f} | '
                  f'{max(endl):7.3f} | {n_nan:3d} | {n_inf:3d} | {n_ninf:4d}{mark}')
        else:
            print(f' {lo:4d}–{lo + sektor:<4d} | {len(werte):3d} |          – |       – |       – | '
                  f'{n_nan:3d} | {n_inf:3d} | {n_ninf:4d}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--topic', default='/cmd_vel_raw', help='Ziel der Drehbefehle')
    p.add_argument('--omega', type=float, default=0.3692, help='Drehrate [rad/s] (kalibriert 9b)')
    p.add_argument('--umdrehungen', type=float, default=1.0)
    p.add_argument('--sektor', type=int, default=15, help='Sektorbreite [°] für die Tabelle')
    p.add_argument('--csv', default='', help='Rohdaten speichern (t, Winkel, range)')
    p.add_argument('--name', default='')
    a = p.parse_args()
    if a.omega <= 0.0 or a.umdrehungen <= 0.0 or not 1 <= a.sektor <= 90:
        p.error('omega und umdrehungen > 0, sektor 1…90')

    rclpy.init()
    node = RangeScan(a)
    node.start_wait = node.now_s()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.step():
                break
    except KeyboardInterrupt:
        node.result = 'Abbruch mit Strg+C'
    finally:
        if rclpy.ok():
            node.stop()
    print(node.result)
    samples = node.samples
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if not samples:
        print('Keine /range-Werte aufgezeichnet.')
        return
    auswerten(samples, a.sektor, a.name)
    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'winkel_deg', 'range_m'])
            w.writerows(samples)
        print(f'Rohdaten: {a.csv}')


if __name__ == '__main__':
    main()

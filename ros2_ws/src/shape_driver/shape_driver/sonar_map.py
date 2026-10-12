"""Ultraschallkarte (Erkundung, Baustein 3): Belegungsraster aus /range und /odom.

Prinzip (Occupancy Grid mit Log-Odds, wie bei Lidar-Karten, aber mit nur EINEM Messstrahl):
- Für jeden /range-Wert wird die Sensorpose aus der aktuellen Roboterpose (/odom) und dem
  Einbauort des Sensors (sensor_x, sensor_y) berechnet.
- Der Schallkegel (±cone_half_deg) wird durch mehrere Strahlen angenähert. Zellen entlang der
  Strahlen bis kurz vor dem Messwert werden als "frei" verstärkt, Zellen am Messwert als "belegt".
- +inf (kein Echo im Messbereich) -> nur "frei" bis free_range. nan/-inf -> ignoriert.
- Werte über max_range werden nur als "frei bis max_range" verwendet (Ultraschall wird mit der
  Entfernung unschärfer, der Kegel breiter).

Ausgänge:
  /map      (nav_msgs/OccupancyGrid, Frame odom, 1 Hz, transient local -> RViz sieht sie sofort)
  Beim Beenden (Strg+C) Speichern nach save_dir:
    karte_<Zeit>.pgm + .yaml  (Format des ROS-map_server: weiß frei, schwarz belegt, grau unbekannt)
    karte_<Zeit>_weg.ppm      (Farbbild: Karte + gefahrener Weg rot + Start grün)

Grenzen: keine Korrektur der Roboterpose durch die Karte (kein SLAM) -> Odometriefehler gehen
direkt in die Karte ein; Bodenecho an Fugen (Stufe 11g) erscheint als Hindernis.
Annahme: cone_half_deg 7,5° (= halber "Messwinkel 15°" gängiger HC-SR04-Datenblätter; ob voller
oder halber Winkel gemeint ist, ist dort nicht eindeutig) – bei Bedarf anpassen.

Stand 09.10.2026: UNGETESTET.
"""
import math
import os
import time
from array import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Range


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SonarMap(Node):
    def __init__(self):
        super().__init__('sonar_map')
        self.declare_parameter('resolution', 0.02)       # m pro Zelle
        self.declare_parameter('size', 6.0)              # m Kantenlänge, Start in der Mitte
        self.declare_parameter('sensor_x', 0.10)         # m vor der Radachse (gemessen 09.10.)
        self.declare_parameter('sensor_y', 0.0)          # m nach links (+) bzw. rechts (−)
        self.declare_parameter('cone_half_deg', 7.5)     # ° halber Öffnungswinkel (Annahme)
        self.declare_parameter('ray_step_deg', 2.5)      # ° Abstand der Teilstrahlen
        self.declare_parameter('min_range', 0.03)        # m
        self.declare_parameter('max_range', 1.5)         # m, darüber nur "frei"
        self.declare_parameter('free_range', 1.0)        # m frei bei +inf
        self.declare_parameter('l_occ', 0.85)            # Log-Odds-Zuwachs belegt
        self.declare_parameter('l_free', 0.4)            # Log-Odds-Abnahme frei
        self.declare_parameter('l_max', 5.0)             # Begrenzung
        self.declare_parameter('odom_timeout', 0.5)      # s
        self.declare_parameter('save_dir', os.path.expanduser('~/messungen/karten'))

        g = lambda n: self.get_parameter(n).value
        self.res = float(g('resolution'))
        self.n = int(round(float(g('size')) / self.res))
        self.origin = -self.n * self.res / 2.0           # Weltkoordinate der Zellecke (0,0)
        self.sx = float(g('sensor_x'))
        self.sy = float(g('sensor_y'))
        half = math.radians(float(g('cone_half_deg')))
        step = math.radians(float(g('ray_step_deg')))
        k = max(0, int(math.floor(half / step + 1e-9)))
        self.ray_offsets = [i * step for i in range(-k, k + 1)] if half > 0 else [0.0]
        self.min_range = float(g('min_range'))
        self.max_range = float(g('max_range'))
        self.free_range = float(g('free_range'))
        self.l_occ = float(g('l_occ'))
        self.l_free = float(g('l_free'))
        self.l_max = float(g('l_max'))
        self.odom_timeout = float(g('odom_timeout'))
        self.save_dir = g('save_dir')

        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)   # [Zeile = y, Spalte = x]
        self.seen = np.zeros((self.n, self.n), dtype=bool)
        self.pose = None            # (x, y, yaw)
        self.pose_time = None
        self.path = []              # Zellen des Wegs
        self.n_ranges = 0
        self.n_used = 0

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Range, '/range', self.on_range, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish_map)
        self.create_timer(10.0, self.log_status)
        self.get_logger().info(
            f'Karte {self.n}×{self.n} Zellen à {self.res * 100:.0f} cm, Sensor bei '
            f'x {self.sx:+.3f} m, y {self.sy:+.3f} m, Kegel ±{math.degrees(half):.1f}° '
            f'({len(self.ray_offsets)} Strahlen), genutzt bis {self.max_range:.2f} m')

    # ---------- Hilfsfunktionen ----------
    def cell(self, x, y):
        return int(math.floor((x - self.origin) / self.res)), int(math.floor((y - self.origin) / self.res))

    def inside(self, cx, cy):
        return 0 <= cx < self.n and 0 <= cy < self.n

    def bresenham(self, x0, y0, x1, y1):
        """Zellen von (x0,y0) bis (x1,y1), ohne Endzelle."""
        cells = []
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
        err = dx + dy
        while (x0, y0) != (x1, y1):
            cells.append((x0, y0))
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return cells

    def update(self, cx, cy, delta):
        if self.inside(cx, cy):
            v = self.logodds[cy, cx] + delta
            self.logodds[cy, cx] = max(-self.l_max, min(self.l_max, v))
            self.seen[cy, cx] = True

    # ---------- Eingänge ----------
    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.pose_time = time.monotonic()
        c = self.cell(self.pose[0], self.pose[1])
        if not self.path or self.path[-1] != c:
            self.path.append(c)

    def on_range(self, msg):
        self.n_ranges += 1
        if self.pose is None or time.monotonic() - self.pose_time > self.odom_timeout:
            return
        r = msg.range
        if math.isnan(r) or (math.isinf(r) and r < 0) or (math.isfinite(r) and r < self.min_range):
            return
        x, y, yaw = self.pose
        # Sensorpose in der Welt
        px = x + self.sx * math.cos(yaw) - self.sy * math.sin(yaw)
        py = y + self.sx * math.sin(yaw) + self.sy * math.cos(yaw)
        hit = math.isfinite(r) and r <= self.max_range
        free_len = (r - self.res) if hit else (self.free_range if math.isinf(r) else self.max_range)
        c0 = self.cell(px, py)
        occupied = set()
        free = set()
        for off in self.ray_offsets:
            a = yaw + off
            if free_len > 0:
                ce = self.cell(px + free_len * math.cos(a), py + free_len * math.sin(a))
                free.update(self.bresenham(c0[0], c0[1], ce[0], ce[1]))
                free.add(ce)
            if hit:
                occupied.add(self.cell(px + r * math.cos(a), py + r * math.sin(a)))
        free -= occupied
        for c in free:
            self.update(c[0], c[1], -self.l_free)
        # Belegt-Gewicht auf die Bogenzellen verteilen (breiter Kegel = unsicherer Ort)
        if occupied:
            w = self.l_occ * min(1.0, 3.0 / len(occupied))
            for c in occupied:
                self.update(c[0], c[1], w)
        self.n_used += 1

    # ---------- Ausgabe ----------
    def grid_values(self):
        """0..100 belegt-Wahrscheinlichkeit, -1 unbekannt."""
        p = 1.0 / (1.0 + np.exp(-self.logodds))
        out = np.round(p * 100.0).astype(np.int8)
        out[~self.seen] = -1
        return out

    def publish_map(self):
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.info.resolution = self.res
        m.info.width = self.n
        m.info.height = self.n
        m.info.origin.position.x = self.origin
        m.info.origin.position.y = self.origin
        m.info.origin.orientation.w = 1.0
        m.data = array('b', self.grid_values().flatten().tobytes())  # zeilenweise, Zeile 0 = kleinstes y
        self.map_pub.publish(m)

    def log_status(self):
        occ = int(np.count_nonzero(self.logodds > 1.0))
        free = int(np.count_nonzero(self.logodds < -1.0))
        self.get_logger().info(
            f'/range: {self.n_ranges} empfangen, {self.n_used} eingetragen; Zellen sicher belegt {occ}, '
            f'sicher frei {free}' + ('' if self.pose is not None else ' – warte auf /odom'))

    def save(self):
        if self.n_used == 0:
            self.get_logger().info('Keine Messwerte eingetragen – nichts gespeichert.')
            return
        os.makedirs(self.save_dir, exist_ok=True)
        stem = os.path.join(self.save_dir, time.strftime('karte_%Y%m%d_%H%M%S'))
        v = self.grid_values()
        img = np.full(v.shape, 205, dtype=np.uint8)                 # unbekannt: grau
        img[(v >= 0) & (v <= 25)] = 254                             # frei: weiß
        img[v >= 65] = 0                                            # belegt: schwarz
        img = np.flipud(img)                                        # Bildzeile 0 = oben = größtes y
        with open(stem + '.pgm', 'wb') as f:
            f.write(f'P5\n{self.n} {self.n}\n255\n'.encode())
            f.write(img.tobytes())
        with open(stem + '.yaml', 'w') as f:
            f.write(f'image: {os.path.basename(stem)}.pgm\nmode: trinary\nresolution: {self.res}\n'
                    f'origin: [{self.origin}, {self.origin}, 0.0]\nnegate: 0\n'
                    f'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
        rgb = np.stack([img, img, img], axis=-1)
        for (cx, cy) in self.path:
            if self.inside(cx, cy):
                rgb[self.n - 1 - cy, cx] = (220, 0, 0)
        if self.path:
            sx, sy = self.path[0]
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    if self.inside(sx + dx, sy + dy):
                        rgb[self.n - 1 - (sy + dy), sx + dx] = (0, 170, 0)
        with open(stem + '_weg.ppm', 'wb') as f:
            f.write(f'P6\n{self.n} {self.n}\n255\n'.encode())
            f.write(rgb.astype(np.uint8).tobytes())
        self.get_logger().info(f'Karte gespeichert: {stem}.pgm/.yaml, {stem}_weg.ppm')


def main(args=None):
    rclpy.init(args=args)
    node = SonarMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

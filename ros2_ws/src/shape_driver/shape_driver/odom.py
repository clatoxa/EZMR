"""Odometrie (Erkundung, Baustein 2): Positionsschätzung aus Radbefehlen und IMU.

Eingänge:
  /wheel_cmd (geometry_msgs/Vector3, vom ESP32, 20 Hz): x = linkes, y = rechtes Rad als
             vorzeichenbehafteter param (ms pro Motorschritt, + vorwärts, 0 = Stopp).
             Das ist der Befehl, den die Motoren TATSÄCHLICH bekommen – nach Firmware-Timeout,
             lokaler Bumper-Sperre und Rundung auf ganze param-Stufen. Deshalb /wheel_cmd und nicht /cmd_vel.
  /imu       (sensor_msgs/Imu): Gierwinkel für die Fahrtrichtung (use_imu = true).

Radgeschwindigkeit: v = (pi * wheel_diameter / steps_per_rev) * 1000 / param  [m/s]
  wheel_diameter 0,0816 m = wirksamer Durchmesser aus Kalibrierfahrt K1 (Stufe 9b, 995 statt 1000 mm).
Richtung: mit IMU der gemessene Gierwinkel (seit Start), sonst aus der Radgeschwindigkeits-Differenz
  (track_width 0,1665 m = wirksame Spurweite aus K2, Stufe 9b).
Driftkorrektur (zupt = true, Stufe 11i): Steht der Roboter sicher still (beide Räder seit still_time
  auf 0 – nach dem Auslaufen), wird die Richtung eingefroren und die IMU-Winkeländerung als
  Gyroskop-Restdrift geschätzt (gleitender Mittelwert, Zeitkonstante bias_tau). Während der Fahrt
  wird diese Drift abgezogen. Änderungen über bias_max (1,5 °/s) gelten als Bewegen von Hand und
  werden nicht gelernt (Test E2: Ausrichten nach dem Start ergab +10 °/s „Drift“). Anlass: Test O1 zeigte im Stillstand −0,31 °/s Drift (Nullpunkt der
  IMU wird nur beim Einschalten bestimmt).
Integration: Radgeschwindigkeiten bleiben bis zur nächsten Nachricht gültig (Motoren laufen im
  Dauerbetrieb weiter); Lücken werden höchstens mit max_gap integriert (Firmware stoppt nach 0,5 s).

Ausgänge:
  /odom      (nav_msgs/Odometry), Frame odom -> base_link
  TF         odom -> base_link
  /odom_path (nav_msgs/Path): gefahrener Weg (neuer Punkt ab path_min_dist bzw. path_min_angle)

Grenzen: keine Radsensoren (Schrittverluste/Schlupf werden nicht erkannt), Fehler summieren sich.

Stand 09.10.2026: Grundfunktion getestet (O1, ohne Driftkorrektur); Driftkorrektur getestet (O2, E1); Plausibilitätsgrenze bias_max UNGETESTET.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Odom(Node):
    def __init__(self):
        super().__init__('odom')
        self.declare_parameter('wheel_diameter', 0.0816)   # m, wirksam (9b, K1)
        self.declare_parameter('track_width', 0.1665)      # m, wirksam (9b, K2) – nur ohne IMU
        self.declare_parameter('steps_per_rev', 2048.0)    # Stufe 7
        self.declare_parameter('use_imu', True)
        self.declare_parameter('max_gap', 0.5)             # s
        self.declare_parameter('zupt', True)               # Drift im Stillstand schätzen/abziehen
        self.declare_parameter('still_time', 1.0)          # s Stillstand, bevor er als sicher gilt
        self.declare_parameter('bias_tau', 5.0)            # s Zeitkonstante der Driftschätzung
        self.declare_parameter('bias_max', 1.5)            # °/s: größere Änderung im Stillstand = von
                                                           # Hand bewegt -> nicht als Drift lernen
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('path_min_dist', 0.01)      # m
        self.declare_parameter('path_min_angle', 5.0)      # °
        self.declare_parameter('path_max_len', 5000)

        g = lambda n: self.get_parameter(n).value
        self.step_dist = math.pi * float(g('wheel_diameter')) / float(g('steps_per_rev'))
        self.track = float(g('track_width'))
        self.use_imu = bool(g('use_imu'))
        self.max_gap = float(g('max_gap'))
        self.zupt = bool(g('zupt'))
        self.still_min = float(g('still_time'))
        self.bias_tau = float(g('bias_tau'))
        self.bias_max = math.radians(float(g('bias_max')))
        self.n_rejected = 0.0          # Stillstandszeit, die wegen Bewegung von Hand verworfen wurde [s]
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')
        self.path_min_dist = float(g('path_min_dist'))
        self.path_min_angle = math.radians(float(g('path_min_angle')))
        self.path_max_len = int(g('path_max_len'))

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/odom_path', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Vector3, '/wheel_cmd', self.on_wheel, qos_profile_sensor_data)
        if self.use_imu:
            self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)

        # Zustand
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0                 # Fahrtrichtung im odom-Frame (0 = Ausrichtung beim Start)
        self.vl = 0.0                  # zuletzt gültige Radgeschwindigkeiten [m/s]
        self.vr = 0.0
        self.last_time = None
        self.imu_yaw = None            # ausgerollter IMU-Winkel
        self.imu_yaw_raw = None
        self.imu_yaw_used = None       # IMU-Winkel beim letzten Integrationsschritt
        self.bias = 0.0                # geschätzte Restdrift [rad/s]
        self.still = 0.0               # Dauer des aktuellen Stillstands [s]
        self.bias_learned = 0.0        # gesamte Stillstandszeit, aus der gelernt wurde [s]
        self.dist = 0.0                # gefahrene Strecke (Betrag)
        self.n_msgs = 0
        self.n_gaps = 0

        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.last_path_pose = None
        self.create_timer(1.0, self.publish_path)
        self.create_timer(10.0, self.log_status)

        self.get_logger().info(
            f'Odometrie aus /wheel_cmd, Schrittweg {self.step_dist * 1000:.4f} mm, '
            f'Richtung {"aus /imu" if self.use_imu else "aus Raddifferenz (Spur %.4f m)" % self.track}')

    def wheel_speed(self, signed_param):
        """Vorzeichenbehafteter param -> Radgeschwindigkeit [m/s]; 0 = Stopp."""
        p = float(signed_param)
        if p == 0.0 or not math.isfinite(p):
            return 0.0
        return math.copysign(self.step_dist * 1000.0 / abs(p), p)

    def on_imu(self, msg):
        y = yaw_from_quat(msg.orientation)
        if self.imu_yaw_raw is None:
            self.imu_yaw = y
            self.imu_yaw_used = y
        else:
            self.imu_yaw += wrap(y - self.imu_yaw_raw)
        self.imu_yaw_raw = y

    def on_wheel(self, msg):
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9
        if self.use_imu and self.imu_yaw is None:
            return                                    # erst mit IMU-Startwinkel beginnen
        if self.last_time is not None:
            dt = t - self.last_time
            if dt > self.max_gap:
                self.n_gaps += 1
                dt = self.max_gap
            if dt > 0.0:
                self.integrate(dt)
        self.last_time = t
        self.vl = self.wheel_speed(msg.x)
        self.vr = self.wheel_speed(msg.y)
        self.n_msgs += 1
        self.publish(now)

    def integrate(self, dt):
        v = 0.5 * (self.vl + self.vr)
        if self.use_imu:
            d_imu = self.imu_yaw - self.imu_yaw_used
            self.imu_yaw_used = self.imu_yaw
            if self.vl == 0.0 and self.vr == 0.0:
                self.still += dt
            else:
                self.still = 0.0
            if self.zupt and self.still >= self.still_min:
                # sicher im Stillstand: Änderung ist Drift -> lernen, Richtung nicht ändern
                rate = d_imu / dt
                if abs(rate) <= self.bias_max:
                    k = min(1.0, dt / self.bias_tau)
                    self.bias += k * (rate - self.bias)
                    self.bias_learned += dt
                else:
                    self.n_rejected += dt          # Roboter wird von Hand gedreht
                d_imu = 0.0
            elif self.zupt:
                d_imu -= self.bias * dt
            yaw_new = self.yaw + d_imu
        else:
            yaw_new = self.yaw + (self.vr - self.vl) / self.track * dt
        yaw_mid = 0.5 * (self.yaw + yaw_new)          # Mittelwert über das Intervall
        ds = v * dt
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw = yaw_new
        self.dist += abs(ds)

    def publish(self, now):
        qx, qy, qz, qw = quat_from_yaw(self.yaw)
        stamp = now.to_msg()
        v = 0.5 * (self.vl + self.vr)
        w = (self.vr - self.vl) / self.track

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = w            # aus Radbefehlen (Sollwert), nicht gemessen
        self.odom_pub.publish(od)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)

        # Wegpunkt anhängen, wenn genug bewegt
        lp = self.last_path_pose
        if (lp is None or math.hypot(self.x - lp[0], self.y - lp[1]) >= self.path_min_dist
                or abs(wrap(self.yaw - lp[2])) >= self.path_min_angle):
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.odom_frame
            ps.pose = od.pose.pose
            self.path.poses.append(ps)
            if len(self.path.poses) > self.path_max_len:
                del self.path.poses[0]
            self.last_path_pose = (self.x, self.y, self.yaw)

    def publish_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

    def log_status(self):
        if self.n_msgs == 0:
            self.get_logger().warn('Noch keine /wheel_cmd-Daten' +
                                   (' bzw. keine /imu-Daten' if self.use_imu else ''))
            return
        self.get_logger().info(
            f'Pose x {self.x:+.3f} m, y {self.y:+.3f} m, Richtung {math.degrees(self.yaw):+.1f}°, '
            f'Strecke {self.dist:.2f} m, Lücken > {self.max_gap:.1f} s: {self.n_gaps}'
            + (f', Drift {math.degrees(self.bias):+.3f} °/s (aus {self.bias_learned:.0f} s Stillstand'
               f'{", %.0f s verworfen" % self.n_rejected if self.n_rejected >= 0.5 else ""})'
               if self.use_imu and self.zupt else ''))


def main(args=None):
    rclpy.init(args=args)
    node = Odom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log_status()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Fährt eine regelmäßige Vielecksfigur (z. B. Viereck) über geometry_msgs/Twist.

Geradeausfahrt zeitgesteuert. Drehung wahlweise
  turn_mode = 'time': Drehdauer = Winkel / angular_speed (wie bisher, ohne Rückführung)
  turn_mode = 'imu' : Drehen, bis der Gierwinkel aus /imu (sensor_msgs/Imu) den Zielwinkel
                      minus turn_lead_deg erreicht (Rückführung, Stufe 11f-c).
Im IMU-Modus wird je Ecke protokolliert: Ziel, Winkel beim Stoppbefehl, Winkel nach settle_time
(Nachdrehen durch Verzögerung und Trägheit).

Alle Fahrwerte sind ROS-Parameter, damit dieselbe Node turtlesim und den realen Roboter ansteuert.
Stand 09.10.2026: turn_mode 'time' getestet (Stufe 2.2, 9); turn_mode 'imu' UNGETESTET.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


def yaw_from_quat(q):
    """Gierwinkel [rad] aus Quaternion (nur Drehung um z ausgewertet)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """Winkel auf (-pi, pi] abbilden."""
    return math.atan2(math.sin(a), math.cos(a))


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
        # Stufe 11f-c: Drehung mit IMU-Rückführung
        self.declare_parameter('turn_mode', 'time')   # 'time' oder 'imu'
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('turn_lead_deg', 0.0)  # früher stoppen (Verzögerung ausgleichen)
        self.declare_parameter('settle_time', 0.5)    # s bis zur Kontrollmessung nach dem Stopp
        self.declare_parameter('imu_timeout', 0.5)    # s ohne /imu -> Abbruch

        topic = self.get_parameter('cmd_vel_topic').value
        self.num_sides = int(self.get_parameter('num_sides').value)
        side_length = float(self.get_parameter('side_length').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.turn_mode = str(self.get_parameter('turn_mode').value)
        self.turn_lead = math.radians(float(self.get_parameter('turn_lead_deg').value))
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)

        self.finished = False
        if (self.num_sides < 3 or side_length <= 0.0 or self.linear_speed <= 0.0
                or self.angular_speed <= 0.0 or rate <= 0.0
                or self.turn_mode not in ('time', 'imu')):
            self.get_logger().error('Ungültige Parameter – Node wird beendet.')
            self.finished = True
            return

        # Phasendauern aus der Geometrie berechnen
        self.turn_angle = 2.0 * math.pi / self.num_sides
        self.forward_duration = side_length / self.linear_speed
        self.turn_duration = self.turn_angle / self.angular_speed

        self.publisher = self.create_publisher(Twist, topic, 10)

        # IMU: fortlaufend „ausgerollter“ Gierwinkel (ohne Sprung bei ±180°)
        self.yaw = None
        self.yaw_raw_last = None
        self.imu_time = None
        if self.turn_mode == 'imu':
            self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.on_imu,
                                     qos_profile_sensor_data)
        self.turn_start_yaw = None
        self.corner_log = []          # (Ecke, Winkel beim Stopp, Winkel nach settle) in Grad
        self.pending_check = None     # (Ecke, Zeitpunkt, Winkel beim Stopp, Startwinkel)

        self.sides_done = 0
        self.done_ticks = 0
        self.state = self.FORWARD
        self.state_start = self.get_clock().now()
        self.waiting_for_imu = self.turn_mode == 'imu'
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f'Start: {self.num_sides} Seiten, Topic {topic}, Fahrzeit {self.forward_duration:.2f} s, '
            f'Drehung {"%.2f s (Zeit)" % self.turn_duration if self.turn_mode == "time" else "per IMU, Vorhalt %.1f°" % math.degrees(self.turn_lead)}')

    # ---------- IMU ----------
    def on_imu(self, msg):
        y = yaw_from_quat(msg.orientation)
        if self.yaw_raw_last is None:
            self.yaw = y
        else:
            self.yaw += wrap(y - self.yaw_raw_last)    # Differenz statt Absolutwert -> kein Sprung
        self.yaw_raw_last = y
        self.imu_time = self.get_clock().now()

    def imu_ok(self):
        if self.imu_time is None:
            return False
        return (self.get_clock().now() - self.imu_time).nanoseconds / 1e9 <= self.imu_timeout

    # ---------- Zustandsautomat ----------
    def elapsed(self):
        """Sekunden seit Beginn des aktuellen Zustands."""
        return (self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def switch_to(self, new_state):
        self.state = new_state
        self.state_start = self.get_clock().now()
        if new_state == self.TURN and self.turn_mode == 'imu':
            self.turn_start_yaw = self.yaw
        self.get_logger().info(f'Zustand -> {new_state} (fertige Seiten: {self.sides_done})')

    def turn_finished(self):
        if self.turn_mode == 'time':
            return self.elapsed() >= self.turn_duration
        turned = self.yaw - self.turn_start_yaw
        if turned >= self.turn_angle - self.turn_lead:
            corner = self.sides_done + 1
            self.pending_check = (corner, self.get_clock().now(), math.degrees(turned),
                                  self.turn_start_yaw)
            return True
        # Sicherheitsgrenze: nie länger als doppelte Solldauer drehen
        if self.elapsed() > 2.0 * self.turn_duration:
            self.get_logger().error('Drehung dauert zu lange – Abbruch (IMU-Werte prüfen)')
            self.switch_to(self.DONE)
        return False

    def update_state(self):
        t = self.elapsed()
        if self.state == self.FORWARD and t >= self.forward_duration:
            self.switch_to(self.TURN)
        elif self.state == self.TURN and self.turn_finished():
            self.sides_done += 1
            if self.sides_done >= self.num_sides:
                self.switch_to(self.DONE)
            else:
                self.switch_to(self.FORWARD)

    def check_settle(self):
        """Winkel settle_time nach dem Stoppbefehl erfassen (Nachdrehen)."""
        if self.pending_check is None:
            return
        corner, t_stop, at_stop, start = self.pending_check
        if (self.get_clock().now() - t_stop).nanoseconds / 1e9 >= self.settle_time:
            after = math.degrees(self.yaw - start)
            self.corner_log.append((corner, at_stop, after))
            self.get_logger().info(
                f'Ecke {corner}: Ziel {math.degrees(self.turn_angle):.1f}°, beim Stopp {at_stop:.2f}°, '
                f'nach {self.settle_time:.1f} s {after:.2f}° (Nachdrehen {after - at_stop:+.2f}°)')
            self.pending_check = None

    def on_timer(self):
        if self.waiting_for_imu:
            if self.imu_ok():
                self.waiting_for_imu = False
                self.state_start = self.get_clock().now()
                self.get_logger().info(f'IMU empfangen (Gier {math.degrees(self.yaw):.1f}°) – Fahrt beginnt')
            else:
                self.publisher.publish(Twist())       # stehen, bis IMU-Daten da sind
                return
        if self.turn_mode == 'imu' and self.state != self.DONE and not self.imu_ok():
            self.get_logger().error('Keine aktuellen /imu-Daten – Abbruch')
            self.switch_to(self.DONE)

        self.update_state()
        self.check_settle()

        msg = Twist()  # alle Felder sind standardmäßig 0.0
        if self.state == self.FORWARD:
            msg.linear.x = self.linear_speed
        elif self.state == self.TURN:
            msg.angular.z = self.angular_speed
        self.publisher.publish(msg)

        # Nach Abschluss einige Nullbefehle senden, Auswertung abwarten, dann beenden
        if self.state == self.DONE:
            self.done_ticks += 1
            if self.done_ticks >= 5 and self.pending_check is None:
                self.timer.cancel()
                self.report()
                self.get_logger().info('Fahrfolge abgeschlossen.')
                self.finished = True

    def report(self):
        if not self.corner_log:
            return
        target = math.degrees(self.turn_angle)
        errors = [after - target for _, _, after in self.corner_log]
        overs = [after - stop for _, stop, after in self.corner_log]
        self.get_logger().info(
            f'Zusammenfassung: Fehler je Ecke (nach Auslaufen) '
            f'{", ".join("%+.2f°" % e for e in errors)}; mittleres Nachdrehen {sum(overs) / len(overs):+.2f}°')


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDriver()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok() and hasattr(node, 'publisher'):
            node.publisher.publish(Twist())   # Sicherheit: Stopp (bei Strg+C greift der ESP32-Timeout)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Zufallsfahrt (Erkundung, Baustein 1): geradeaus, bei Hindernis/Bumper ausweichen.

Ablauf (Zustandsautomat, 20 Hz):
  FORWARD  geradeaus fahren, bis obstacle_stop HINDERNIS (2) oder BUMPER (4) meldet
  BACKUP   kurz rückwärts (backup_time) – nur wenn vorher ein Stück gefahren wurde
  danach Wahl der Drehrichtung:
    1. Bumper (Stufe 11i, E7): Seite aus /bumper (Bit 0 links, Bit 1 rechts) -> von der Seite WEG drehen
       (links getroffen -> rechts drehen und umgekehrt). Beide Seiten -> wie Ultraschall-Hindernis.
    2. Ultraschall-Hindernis mit look_around = true: UMSCHAUEN – auf +look_angle drehen, messen,
       auf −look_angle drehen, messen (erste Seite zufällig); dann zur Seite mit mehr Platz.
       Beide Seiten < look_min_free -> weiter bis zur Rückrichtung (Wende).
    3. sonst: Zufallswinkel turn_min_deg…turn_max_deg, Richtung zufällig.
  TURN     IMU-geregelte Drehung mit Vorhalt turn_lead_deg (wie shape_driver, 11f-c)
  SETTLE   settle_time stehen (Nachdrehen abklingen lassen)
  MEASURE  (nur Umschauen) look_time lang /range sammeln, Median bilden

Sicherheit:
- Fährt über /cmd_vel_raw; obstacle_stop bleibt einziger Publisher auf /cmd_vel und sperrt
  Vorwärtsfahrt bei Hindernis/Bumper ohnehin selbst. Diese Node reagiert nur darauf.
- Ohne aktuelle /robot_state- oder /imu-Daten (> data_timeout) wird angehalten (Nullbefehl).
- Rückwärts gibt es keinen Sensor: backup_time klein halten (Standard 1 s ≈ 3 cm).
- Ecken (E2): ist der Weg direkt nach einer Drehung wieder versperrt, wird in DIESELBE Richtung
  weitergedreht, statt links/rechts zu pendeln (gilt nicht, wenn die Bumper-Seite bekannt ist).
- max_forward_min/max > 0 (E3): nach zufälliger Fahrzeit auch ohne Hindernis abbiegen.
- duration > 0: nach dieser Zeit anhalten und beenden; 0 = bis Strg+C.

/robot_state (std_msgs/UInt8, von obstacle_stop): 1 FREI, 2 HINDERNIS, 3 keine Daten, 4 BUMPER.
Drehrichtung: positiv = gegen den Uhrzeigersinn = nach LINKS (REP 103).

Stand 12.10.2026: Grundfunktion, Eckenregel, freiwilliger Richtungswechsel getestet (W1, E1–E5);
Bumper-Seite und Umschauen (look_around) UNGETESTET.
"""
import math
import random
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import UInt8

STATE_FREE = 1
STATE_OBSTACLE = 2
STATE_NO_DATA = 3
STATE_BUMPER = 4
BUMPER_LEFT = 1
BUMPER_RIGHT = 2


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Wander(Node):
    FORWARD = 'FORWARD'
    BACKUP = 'BACKUP'
    TURN = 'TURN'
    SETTLE = 'SETTLE'
    MEASURE = 'MEASURE'
    WAIT = 'WAIT'          # keine Daten -> stehen

    def __init__(self):
        super().__init__('wander')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')
        self.declare_parameter('linear_speed', 0.03129)    # m/s, kalibriert (9b)
        self.declare_parameter('angular_speed', 0.3692)    # rad/s, kalibriert (9b)
        self.declare_parameter('turn_min_deg', 90.0)
        self.declare_parameter('turn_max_deg', 180.0)
        self.declare_parameter('turn_lead_deg', 4.0)       # Vorhalt (11f-c)
        self.declare_parameter('backup_time', 1.0)         # s rückwärts
        self.declare_parameter('min_forward_time', 0.5)    # s; kürzer gefahren -> ohne Zurücksetzen erneut drehen
        self.declare_parameter('max_forward_min', 0.0)     # s; > 0: nach zufälliger Fahrzeit zwischen
        self.declare_parameter('max_forward_max', 0.0)     #    min und max freiwillig abbiegen (0 = aus)
        self.declare_parameter('look_around', False)       # Umschauen bei Ultraschall-Hindernis
        self.declare_parameter('look_angle_deg', 90.0)     # Blickwinkel links/rechts
        self.declare_parameter('look_time', 0.6)           # s Messdauer je Seite (≈ 6 Werte bei 10 Hz)
        self.declare_parameter('look_min_free', 0.35)      # m: darunter gilt eine Seite als zu
        self.declare_parameter('settle_time', 0.5)         # s
        self.declare_parameter('data_timeout', 1.0)        # s
        self.declare_parameter('duration', 120.0)          # s; 0 = unbegrenzt
        self.declare_parameter('seed', -1)                 # >= 0: reproduzierbare Zufallsfolge
        self.declare_parameter('rate', 20.0)               # Hz

        g = lambda n: self.get_parameter(n).value
        self.v = float(g('linear_speed'))
        self.w = float(g('angular_speed'))
        self.turn_min = math.radians(float(g('turn_min_deg')))
        self.turn_max = math.radians(float(g('turn_max_deg')))
        self.lead = math.radians(float(g('turn_lead_deg')))
        self.backup_time = float(g('backup_time'))
        self.min_forward = float(g('min_forward_time'))
        self.fwd_min = float(g('max_forward_min'))
        self.fwd_max = max(self.fwd_min, float(g('max_forward_max')))
        self.look_around = bool(g('look_around'))
        self.look_angle = math.radians(float(g('look_angle_deg')))
        self.look_time = float(g('look_time'))
        self.look_min_free = float(g('look_min_free'))
        self.settle_time = float(g('settle_time'))
        self.data_timeout = float(g('data_timeout'))
        self.duration = float(g('duration'))
        seed = int(g('seed'))
        self.rng = random.Random(seed if seed >= 0 else None)

        self.finished = False
        if (self.v <= 0.0 or self.w <= 0.0 or not 0.0 < self.turn_min <= self.turn_max
                or self.backup_time < 0.0 or float(g('rate')) <= 0.0
                or not 0.0 < self.look_angle <= math.pi or self.look_time <= 0.0):
            self.get_logger().error('Ungültige Parameter – Node wird beendet.')
            self.finished = True
            return

        self.pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
        self.create_subscription(UInt8, '/robot_state', self.on_state, 10)
        self.create_subscription(UInt8, '/bumper', self.on_bumper, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Range, '/range', self.on_range, qos_profile_sensor_data)

        self.robot_state = None
        self.state_time = None
        self.yaw = None
        self.yaw_raw_last = None
        self.imu_time = None
        self.bumper_hit = 0              # zuletzt gemeldete Bumper-Bits != 0
        self.bumper_hit_time = None

        self.mode = self.WAIT
        self.mode_start = self.now()
        self.start_time = None           # Beginn der Fahrt (erste gültige Daten)
        self.turn_target = 0.0           # vorzeichenbehaftet [rad]
        self.turn_start_yaw = 0.0
        self.turn_then = self.FORWARD    # Zustand nach der Drehung (FORWARD oder MEASURE)
        self.pending_sign = 0.0          # nach BACKUP: festgelegte Richtung (Bumper), 0 = offen
        self.fwd_limit = 0.0
        self.samples = []                # /range-Werte während MEASURE
        self.look = []                   # [(Vorzeichen der Seite, Median)]
        self.look_first = 1.0
        # Statistik
        self.n_obstacle = 0
        self.n_bumper = 0
        self.n_turns = 0
        self.n_voluntary = 0
        self.n_bumper_side = 0
        self.n_look = 0
        self.forward_total = 0.0

        self.timer = self.create_timer(1.0 / float(g('rate')), self.on_timer)
        self.get_logger().info(
            f'Zufallsfahrt: v {self.v:.4f} m/s, Drehung {g("turn_min_deg"):.0f}–{g("turn_max_deg"):.0f}° '
            f'(Vorhalt {g("turn_lead_deg"):.1f}°), Umschauen {"an" if self.look_around else "aus"}, '
            f'Dauer {"unbegrenzt" if self.duration <= 0 else "%.0f s" % self.duration}'
            f'{", seed %d" % seed if seed >= 0 else ""}')

    # ---------- Eingänge ----------
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_state(self, msg):
        self.robot_state = msg.data
        self.state_time = self.now()

    def on_bumper(self, msg):
        if msg.data:
            self.bumper_hit = msg.data
            self.bumper_hit_time = self.now()

    def on_range(self, msg):
        if self.mode == self.MEASURE:
            r = msg.range
            if math.isnan(r) or (math.isinf(r) and r < 0):
                r = 0.0                      # kein Echo / zu nah -> als "zu" werten
            elif math.isinf(r):
                r = 4.0                      # frei bis Messbereichsende
            self.samples.append(r)

    def on_imu(self, msg):
        y = yaw_from_quat(msg.orientation)
        if self.yaw_raw_last is None:
            self.yaw = y
        else:
            self.yaw += wrap(y - self.yaw_raw_last)
        self.yaw_raw_last = y
        self.imu_time = self.now()

    def data_ok(self):
        t = self.now()
        return (self.state_time is not None and t - self.state_time <= self.data_timeout
                and self.imu_time is not None and t - self.imu_time <= self.data_timeout)

    # ---------- Zustände ----------
    def elapsed(self):
        return self.now() - self.mode_start

    def set_mode(self, mode):
        if self.mode == self.FORWARD:
            self.forward_total += self.elapsed()
        self.mode = mode
        self.mode_start = self.now()
        if mode == self.FORWARD:
            self.fwd_limit = self.rng.uniform(self.fwd_min, self.fwd_max) if self.fwd_min > 0.0 else 0.0
        elif mode == self.MEASURE:
            self.samples = []

    def turn(self, angle, then=None, reason=''):
        """Drehung um angle [rad] (+ = links) starten; danach Zustand then."""
        self.turn_target = angle
        self.turn_start_yaw = self.yaw
        self.turn_then = then or self.FORWARD
        self.n_turns += 1
        self.get_logger().info(f'Drehe {math.degrees(angle):+.0f}°{" (" + reason + ")" if reason else ""}')
        self.set_mode(self.TURN)

    def random_angle(self):
        return self.rng.uniform(self.turn_min, self.turn_max)

    def bumper_sign(self):
        """Drehrichtung weg von der getroffenen Seite; 0 = unbekannt oder beide Seiten."""
        if self.bumper_hit_time is None or self.now() - self.bumper_hit_time > 2.0:
            return 0.0
        if self.bumper_hit == BUMPER_LEFT:
            return -1.0                       # links getroffen -> nach rechts drehen
        if self.bumper_hit == BUMPER_RIGHT:
            return 1.0                        # rechts getroffen -> nach links drehen
        return 0.0

    def avoid(self, immediate):
        """Ausweichen nach Hindernis/Bumper (nach BACKUP bzw. sofort bei immediate)."""
        sign = self.pending_sign
        self.pending_sign = 0.0
        if sign != 0.0:
            self.n_bumper_side += 1
            self.turn(sign * self.random_angle(),
                      reason='Bumper ' + ('links -> rechts' if sign < 0 else 'rechts -> links'))
        elif immediate and self.turn_target != 0.0:
            # Ecke: gleich nach einer Drehung wieder blockiert -> gleiche Richtung weiter
            self.turn(math.copysign(self.random_angle(), self.turn_target), reason='Ecke')
        elif self.look_around:
            self.n_look += 1
            self.look = []
            self.look_first = self.rng.choice((1.0, -1.0))
            self.turn(self.look_first * self.look_angle, then=self.MEASURE, reason='Umschauen 1')
        else:
            self.turn(self.rng.choice((1.0, -1.0)) * self.random_angle())

    def side_name(self, sign):
        return 'links' if sign > 0 else 'rechts'

    def after_measure(self):
        med = statistics.median(self.samples) if self.samples else 0.0
        side = self.look_first if not self.look else -self.look_first
        self.look.append((side, med))
        if len(self.look) == 1:
            # zur anderen Seite (2 × Blickwinkel zurück)
            self.turn(-2.0 * self.look_first * self.look_angle, then=self.MEASURE, reason='Umschauen 2')
            return
        (s1, r1), (s2, r2) = self.look
        txt = f'{self.side_name(s1)} {r1:.2f} m, {self.side_name(s2)} {r2:.2f} m'
        if max(r1, r2) < self.look_min_free:
            self.get_logger().info(f'Umschauen: {txt} -> beide zu, wenden')
            # weiter in Richtung der letzten Drehung bis zur Rückrichtung
            self.turn(s2 * (math.pi - self.look_angle), reason='Wende')
        elif r2 >= r1:
            self.get_logger().info(f'Umschauen: {txt} -> {self.side_name(s2)}')
            self.set_mode(self.FORWARD)          # steht schon in Richtung s2
        else:
            self.get_logger().info(f'Umschauen: {txt} -> {self.side_name(s1)}')
            self.turn(2.0 * s1 * self.look_angle, reason='zur freieren Seite')

    def blocked(self):
        return self.robot_state in (STATE_OBSTACLE, STATE_BUMPER)

    def on_timer(self):
        t = self.now()
        if not self.data_ok():
            if self.mode != self.WAIT:
                self.get_logger().warn('Keine aktuellen /robot_state- oder /imu-Daten – halte an '
                                       '(läuft obstacle_stop?)')
                self.set_mode(self.WAIT)
            self.pub.publish(Twist())
            return
        if self.robot_state == STATE_NO_DATA:
            # obstacle_stop selbst hat keine Sensordaten -> vorwärts ohnehin gesperrt; stehen
            if self.mode != self.WAIT:
                self.get_logger().warn('obstacle_stop meldet: keine Sensordaten – halte an')
                self.set_mode(self.WAIT)
            self.pub.publish(Twist())
            return

        if self.mode == self.WAIT:
            if self.start_time is None:
                self.start_time = t
                self.get_logger().info('Daten vorhanden – Fahrt beginnt')
            else:
                self.get_logger().info('Daten wieder vorhanden – weiter')
            self.set_mode(self.FORWARD)

        if self.duration > 0.0 and t - self.start_time >= self.duration:
            self.finish('Fahrzeit erreicht')
            return

        msg = Twist()
        if self.mode == self.FORWARD:
            if self.blocked():
                driven = self.elapsed()
                if self.robot_state == STATE_BUMPER:
                    self.n_bumper += 1
                    self.pending_sign = self.bumper_sign()
                    bits = {1: 'links', 2: 'rechts', 3: 'beide'}.get(self.bumper_hit, '?')
                    self.get_logger().warn(f'BUMPER ({bits}) nach {driven:.1f} s Fahrt')
                else:
                    self.n_obstacle += 1
                    self.pending_sign = 0.0
                    self.get_logger().info(f'Hindernis nach {driven:.1f} s Fahrt')
                if driven >= self.min_forward and self.backup_time > 0.0:
                    self.set_mode(self.BACKUP)
                else:
                    self.avoid(immediate=True)
            elif self.fwd_limit > 0.0 and self.elapsed() >= self.fwd_limit:
                # freiwilliger Richtungswechsel: verhindert Pendeln auf einer Linie (Test E3)
                self.n_voluntary += 1
                self.get_logger().info(f'Richtungswechsel nach {self.elapsed():.1f} s (ohne Hindernis)')
                self.turn(self.rng.choice((1.0, -1.0)) * self.random_angle())
            else:
                msg.linear.x = self.v
        if self.mode == self.BACKUP:
            if self.elapsed() >= self.backup_time:
                self.avoid(immediate=False)
            else:
                msg.linear.x = -self.v
        if self.mode == self.TURN:
            # Fortschritt in Drehrichtung (negativ, falls der Roboter falsch herum dreht)
            progress = (self.yaw - self.turn_start_yaw) * math.copysign(1.0, self.turn_target)
            if progress >= abs(self.turn_target) - self.lead:
                self.set_mode(self.SETTLE)
            elif self.elapsed() > 2.0 * abs(self.turn_target) / self.w + 2.0:
                self.get_logger().error('Drehung dauert zu lange – breche Drehung ab')
                self.set_mode(self.SETTLE)
            else:
                msg.angular.z = math.copysign(self.w, self.turn_target)
        if self.mode == self.SETTLE:
            if self.elapsed() >= self.settle_time:
                turned = math.degrees(self.yaw - self.turn_start_yaw)
                self.get_logger().info(f'Drehung fertig: {turned:+.1f}° (Ziel {math.degrees(self.turn_target):+.0f}°)')
                self.set_mode(self.turn_then)
        if self.mode == self.MEASURE:
            if self.elapsed() >= self.look_time:
                self.after_measure()
        self.pub.publish(msg)

    def finish(self, reason):
        self.set_mode(self.WAIT)
        for _ in range(5):
            self.pub.publish(Twist())
        total = self.now() - self.start_time if self.start_time else 0.0
        self.get_logger().info(
            f'{reason}. Zusammenfassung: {total:.0f} s, davon {self.forward_total:.0f} s geradeaus '
            f'(≈ {self.forward_total * self.v:.2f} m), {self.n_obstacle} Hindernisse, '
            f'{self.n_bumper} Bumper, {self.n_turns} Drehungen, davon {self.n_voluntary} freiwillig; '
            f'Bumper-Seite genutzt {self.n_bumper_side}×, umgeschaut {self.n_look}×')
        self.timer.cancel()
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = Wander()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok() and hasattr(node, 'pub'):
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

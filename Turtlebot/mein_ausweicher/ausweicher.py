import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class HindernisAusweicher(Node):
    def __init__(self):
        super().__init__('hindernis_ausweicher')
        self.declare_parameter('grenze', 0.5)
        self.declare_parameter('dreh_dauer', 1.5)
        self.declare_parameter('fahr_tempo', 0.15)
        self.declare_parameter('dreh_tempo', 0.6)
        self.fahr_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_empfangen, 10)
        self.zustand = 'FAHREN'
        self.weg_frei = False
        self.dreh_ticks = 0
        self.timer = self.create_timer(0.1, self.arbeite)

    def scan_empfangen(self, msg):
        grenze = self.get_parameter('grenze').get_parameter_value().double_value

        # Exakt den Index fuer "geradeaus vorne" (0 rad) berechnen
        # index = (0 - angle_min) / angle_increment
        vorne_index = int((0.0 - msg.angle_min) / msg.angle_increment)
        n = len(msg.ranges)
        vorne_index = vorne_index % n  # sicherheitshalber in gueltigen Bereich

        fenster = 30
        werte = []
        for offset in range(-fenster, fenster + 1):
            i = (vorne_index + offset) % n
            d = msg.ranges[i]
            if d > 0.05 and math.isfinite(d):
                werte.append(d)

        if len(werte) < 5:
            self.weg_frei = False
            self.get_logger().info('Zu wenige gueltige Punkte -> stoppe sicherheitshalber')
            return

        werte.sort()
        naechste = werte[:10]
        naehester = naechste[len(naechste) // 2]

        self.weg_frei = naehester > grenze
        self.get_logger().info(
            f'vorne_index={vorne_index}  Frontabstand: {naehester:.2f} m  -> weg_frei={self.weg_frei}')

    def arbeite(self):
        befehl = Twist()
        fahr_tempo = self.get_parameter('fahr_tempo').get_parameter_value().double_value
        dreh_tempo = self.get_parameter('dreh_tempo').get_parameter_value().double_value
        dreh_dauer = self.get_parameter('dreh_dauer').get_parameter_value().double_value
        dreh_ticks_soll = int(dreh_dauer / 0.1)

        if self.zustand == 'FAHREN':
            if self.weg_frei:
                befehl.linear.x = fahr_tempo
            else:
                self.zustand = 'DREHEN'
                self.dreh_ticks = 0
        elif self.zustand == 'DREHEN':
            if self.dreh_ticks < dreh_ticks_soll:
                befehl.angular.z = dreh_tempo
                self.dreh_ticks += 1
            else:
                self.zustand = 'FAHREN'

        self.fahr_publisher.publish(befehl)


def main(args=None):
    rclpy.init(args=args)
    node = HindernisAusweicher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

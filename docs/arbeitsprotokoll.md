# Arbeitsprotokoll – Praktikum „Echtzeitsysteme und mobile Robotik“

Laufende Dokumentation aller Arbeitsschritte als Grundlage für Beleg und Repository.
Es werden nur tatsächlich durchgeführte und überprüfte Ergebnisse eingetragen.
Offene oder ungeprüfte Punkte sind als solche gekennzeichnet.

**Legende:** ✔ nachgewiesen · ⏳ offen · ⚠ Problem aufgetreten

---

## Projektübersicht

| Abschnitt | Inhalt | Status |
|---|---|---|
| 2.1 | ROS-2-Grundlagen, turtlesim | ✔ im Kern abgeschlossen |
| 2.2 | Eigenes ROS-2-Paket (Fahrfolge in turtlesim) | ⏳ |
| 2.3 | ESP32-Roboter über micro-ROS | ⏳ |
| 2.4 | Sensorprojekt (Erfassung ESP32, Auswertung Host) | ⏳ |
| 2.5 | TurtleBot 4 | ausgeschlossen |
| 2.6 / OTA | FreeRTOS, OTA-Updates | optional, nicht ausgewählt |

---

## Systemumgebung (Host-PC)

| Komponente | Wert |
|---|---|
| Rechner | MS-7C02 |
| Betriebssystem | Ubuntu 22.04.5 LTS (jammy), x86_64, Dual-Boot |
| Locale | de_DE.UTF-8 |
| ROS 2 | Humble, `ros-humble-desktop` 0.10.0-1jammy.20260908.101832 |
| Paketquelle | `ros2-apt-source` 1.3.0~jammy |
| Build-Werkzeuge | `ros-dev-tools` (installiert; `which colcon` noch zu bestätigen ⏳) |
| Kernel-Version | ⏳ noch eintragen (`uname -r`) |

**Entscheidung Arbeitsumgebung:** Auf dem Host lief zuvor Ubuntu 24.04. ROS 2 Humble wird offiziell nur für Ubuntu 22.04 als Binärpaket bereitgestellt; die Praktikumsanleitung (2.1) fordert Ubuntu 22.04. Daher wurde Ubuntu 22.04 nativ als Dual-Boot eingerichtet. Begründung gegenüber einer VM: direkter USB-Zugriff auf den ESP32 und direkte Netzwerkerreichbarkeit des micro-ROS-Agents in Abschnitt 2.3.

---

## Protokoll

### 03.10.2026 – Installation ROS 2 Humble

**Ziel:** ROS 2 Humble installieren und Kommunikation verifizieren.

**Vorgehen:** Installation nach offizieller Humble-Dokumentation (deb-Pakete):
1. Ubuntu-Version und Locale geprüft (`lsb_release -a`, `locale`) ✔
2. Universe-Quelle aktiviert, ROS-2-Paketquelle über `ros2-apt-source` eingerichtet ✔
3. `sudo apt update`, `sudo apt upgrade` ✔
4. `ros-humble-desktop` und `ros-dev-tools` installiert; `/opt/ros/humble` vorhanden ✔

**⚠ Problem:** `E: Paket ros-humble-desktop kann nicht gefunden werden`
- Ursache: Nach dem Einrichten der Paketquelle wurde kein `apt update` ausgeführt; der Paketindex von `packages.ros.org` war lokal nicht vorhanden.
- Prüfung: `/etc/apt/sources.list.d/ros2.sources` korrekt (URI, Suite `jammy`, Signed-By).
- Lösung: `sudo apt update` – Quelle `packages.ros.org/ros2/ubuntu jammy` wurde geladen, danach Installation erfolgreich.
- Erkenntnis: Neue apt-Quellen sind erst nach `apt update` nutzbar.

**Ergebnis talker/listener ✔**
- `ros2 run demo_nodes_cpp talker` und `ros2 run demo_nodes_py listener` kommunizieren.
- `ros2 topic info /chatter`: Typ `std_msgs/msg/String`, 1 Publisher, 1 Subscriber.
- Damit sind C++- (rclcpp) und Python-Schnittstelle (rclpy) funktionsfähig.
- Nachweis: Screenshot beider Terminals.

### 03.10.2026 – ROS-Umgebung dauerhaft laden

**Beobachtung:** In einem neuen Terminal ohne `source` → `ros2: Befehl nicht gefunden`.

**Erkenntnis:** `source /opt/ros/humble/setup.bash` setzt die Umgebungsvariablen nur für das jeweilige Terminal.

**Lösung:** Zeile `source /opt/ros/humble/setup.bash` an `~/.bashrc` angehängt.

**Ergebnis ✔:** In einem neu geöffneten Terminal liefert `printenv ROS_DISTRO` → `humble`.
`~/.bashrc` wird nur beim Start eines Terminals gelesen; im bereits offenen Terminal war die Variable zunächst leer.

### 03.10.2026 – turtlesim: ROS-Graph untersuchen

**Vorgehen:** `turtlesim_node` und `turtle_teleop_key` gestartet; Analyse mit `ros2 node list`, `ros2 topic list -t`, `ros2 node info /turtlesim`.

**⚠ Zwischenfall:** Zunächst lief nur `teleop_turtle`; `/turtle1/cmd_vel` war trotzdem gelistet.
Erkenntnis: Ein Topic erscheint, sobald mindestens ein Publisher **oder** Subscriber existiert. Die Sichtbarkeit belegt nicht, dass ein Empfänger läuft (Prüfung über `ros2 topic info` / `ros2 node list`).

**Ergebnis ✔ – Schnittstellen von `/turtlesim`:**

| Art | Name | Typ |
|---|---|---|
| Subscriber | `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` |
| Publisher | `/turtle1/pose` | `turtlesim/msg/Pose` |
| Publisher | `/turtle1/color_sensor` | `turtlesim/msg/Color` |
| Services | u. a. `/reset`, `/spawn`, `/clear`, `/kill`, `/turtle1/set_pen`, `/turtle1/teleport_absolute`, `/turtle1/teleport_relative` | – |
| Action | `/turtle1/rotate_absolute` | `turtlesim/action/RotateAbsolute` |

**Erkenntnisse:**
- Die Anleitung (2.2) nennt `/cmd_vel`, turtlesim verwendet `/turtle1/cmd_vel` → Topicname der eigenen Node muss konfigurierbar sein (Parameter oder Remapping).
- teleop nutzt zwei Kommunikationsarten: Pfeiltasten → Topic (`Twist`), Buchstabentasten → Action (`RotateAbsolute`).
- Neue Action-Ziele brechen laufende Ziele ab (beobachtet: `WARN … Aborting previous goal`).
- Kommunikationsarten: Topic = kontinuierlicher Datenstrom; Service = Anfrage/Antwort; Action = Ziel mit Feedback, Ergebnis und Abbruchmöglichkeit.

### 03.10.2026 – Versuch: Wirkdauer eines Bewegungsbefehls

**Fragestellung:** Wie lange wirkt ein einzelner `Twist`-Befehl auf `/turtle1/cmd_vel`?

**Relevante Felder** (`ros2 interface show geometry_msgs/msg/Twist`): `linear.x` (vorwärts/rückwärts), `angular.z` (Drehung um Hochachse, rad/s, positiv = gegen den Uhrzeigersinn).

**Vorgehen:** `linear.x = 2.0`, `angular.z = 1.8`
- (a) einzelner Befehl: `ros2 topic pub --once …`
- (b) wiederholter Befehl: `ros2 topic pub --rate 1 …`
- Beobachtung über Simulationsfenster und `ros2 topic echo /turtle1/pose`

**Beobachtung:**
- (a) Turtle fährt nur ein Bogenstück und bleibt stehen.
- (b) Turtle fährt fortlaufend einen Kreis; Geschwindigkeitswerte in `/turtle1/pose` ändern sich entsprechend.

**Erklärung:** turtlesim setzt die Geschwindigkeit auf 0, wenn der letzte Befehl älter als 1,0 s ist (Quellcode `turtlesim/src/turtle.cpp`, Branch `humble`).

**Folgerungen:**
- Eigene Fahr-Node muss Befehle periodisch senden (Timer, z. B. 10 Hz); 1 Hz liegt genau an der Timeout-Grenze.
- Für den realen Roboter (2.3) ist ein vergleichbares Timeout-/Stoppverhalten in der ESP32-Firmware vorzusehen.
- Die Fahrfolge ist zeitgesteuert (Steuerung ohne Positionsrückführung, keine Regelung).
- `/turtle1/pose` liefert Simulationseinheiten, keine realen Meter.

**Offen ⏳:** Stoppverhalten nach Abbruch von (b) mit Strg+C noch nicht bestätigt.

---

## Offene Punkte

- [ ] `which colcon` prüfen (Bestätigung `ros-dev-tools`)
- [ ] Kernel-Version eintragen (`uname -r`)
- [ ] Stoppverhalten nach Abbruch von `--rate 1` bestätigen
- [ ] rqt_graph-Screenshot mit vollständiger Kette (Nodes/Topics (all), Hide-Filter aus) – optional
- [ ] Abschnitt 2.2: Workspace anlegen, Python-Paket mit Fahr-Node (Viereck)

## Quellen

- Praktikumsanleitung „Praktikum_2026“, Abschnitte 2.1, 4.1.1, 4.1.2
- ROS 2 Humble – Installation Ubuntu (deb packages): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- turtlesim Quellcode (Humble): https://github.com/ros/ros_tutorials/blob/humble/turtlesim/src/turtle.cpp

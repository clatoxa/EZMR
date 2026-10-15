# EZMR – Echtzeitsysteme und mobile Robotik

Quellcode und Dokumentation zum Praktikum „Echtzeitsysteme und mobile Robotik“ (HTWK Leipzig).
Das Repository gehört zum schriftlichen Beleg und enthält drei Teile:

| Teil | Inhalt | Verzeichnisse |
|---|---|---|
| **A** | Eigener ESP32-Roboter mit ROS 2 Humble und micro-ROS (Anleitung 2.1–2.4, optional 2.6 FreeRTOS und OTA) | `ros2_ws/`, `firmware/`, `tools/`, `docs/` |
| **B** | Praktikum mit dem TurtleBot 4 | `Turtlebot/` |
| **C** | Programmieraufgaben in C (TCP-Client/Server) | `Programmieraufgaben/` |

Ausführliche Beschreibung, Messergebnisse und Bewertung: siehe Beleg. Für Teil A sind alle Versuche
mit Messwerten, Fehlern und Lösungen in [`docs/arbeitsprotokoll.md`](docs/arbeitsprotokoll.md) festgehalten.

---

## Teil A – ESP32-Roboter mit ROS 2 und micro-ROS

### Was der Roboter kann

- **Viereck/Vieleck fahren** – dieselbe Node wie in turtlesim, nur mit anderer Parameterdatei; kalibriert (Endpunktfehler ≈ 3 mm), Drehungen wahlweise zeit- oder IMU-geregelt.
- **Hinderniserkennung** (Sensorprojekt 2.4): Ultraschall und Bumper werden auf dem ESP32 erfasst, auf dem Host ausgewertet; ein Sicherheitsfilter sperrt die Vorwärtsfahrt, ein Buzzer warnt in Abstandsstufen, ein OLED-Display zeigt den Zustand.
- **Erkundungsfahrt mit Ultraschallkarte**: Zufallsfahrt mit Ausweichen (Bumper-Seite, „Umschauen“ links/rechts), Odometrie aus Radbefehlen und IMU mit Driftkorrektur, Belegungskarte in RViz2, Speichern als Bild.
- **FreeRTOS** (2.6): Tasks für Display, Bumper und IMU auf Kern 0, Mutexe, Semaphore aus Interrupt, Task-Notification, periodische Abtastung mit `vTaskDelayUntil`.
- **OTA** (4.1.4): Firmware-Update über WLAN, Motorstopp bei Update-Beginn.

### Systemaufbau

```
 HOST-PC (Ubuntu 22.04, ROS 2 Humble)                   ESP32 (Arduino, micro-ROS, FreeRTOS)
 wander / shape_driver_node
        │ /cmd_vel_raw
        ▼
 obstacle_stop ── /cmd_vel ─────► micro-ROS-Agent ───►  Motoren (UART → 2× ATtiny212, Schrittmotoren)
   ▲      └──── /robot_state ───► (UDP über WLAN) ───►  OLED-Display (Task)
   │ /range, /bumper ◄────────────────────────────────  Ultraschall HC-SR04, Bumper (Interrupt → Task)
 distance_warner ── /buzzer ──────────────────────────►  Buzzer
 odom ◄──────── /wheel_cmd, /imu ◄────────────────────  Radbefehle, IMU MPU-6500 (Task, 100 Hz)
   │ /odom, TF odom→base_link, /odom_path               /imu_reset ──► Gyro-Nullpunkt neu messen
   ▼
 sonar_map ── /map ──► RViz2 (config/explore.rviz)
```

- **Erfassung auf dem ESP32, Auswertung auf dem Host.** Der ESP32 misst und setzt Fahrbefehle um; Entscheidungen (stoppen, ausweichen, kartieren) trifft der Host.
- **`obstacle_stop` ist der einzige Publisher auf `/cmd_vel`.** Fahr-Nodes senden auf `/cmd_vel_raw`.
- **Fail-safe:** ohne aktuelle Sensordaten wird nicht vorwärts gefahren; ohne Fahrbefehl (> 0,5 s) stoppt der ESP32 selbst; bei gedrücktem Bumper sperrt auch die Firmware lokal.

### Topics

| Topic | Typ | Richtung | Inhalt |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Host → ESP32 | Fahrbefehl (nur von `obstacle_stop`) |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | Host intern | Fahrbefehl der Fahr-Nodes vor dem Filter |
| `/range` | `sensor_msgs/Range` | ESP32 → Host | Ultraschall, 10 Hz (best effort) |
| `/bumper` | `std_msgs/UInt8` | ESP32 → Host | Bit 0 links, Bit 1 rechts |
| `/imu` | `sensor_msgs/Imu` | ESP32 → Host | Gierwinkel, Drehraten, Beschleunigung, 20 Hz (best effort) |
| `/wheel_cmd` | `geometry_msgs/Vector3` | ESP32 → Host | tatsächlich an die Motoren gesendete Befehle (best effort) |
| `/robot_state` | `std_msgs/UInt8` | Host → ESP32 | 1 FREI, 2 HINDERNIS, 3 keine Daten, 4 BUMPER (Display) |
| `/buzzer` | `std_msgs/UInt16` | Host → ESP32 | Pieptakt in ms, 0 aus, 1 Dauerton |
| `/imu_reset` | `std_msgs/UInt8` | Host → ESP32 | 1 = Gyro-Nullpunkt neu messen und Winkel 0 (nur im Stillstand) |
| `/loop_diag` | `geometry_msgs/Vector3` | ESP32 → Host | Laufzeitdiagnose der Hauptschleife |
| `/odom`, `/odom_path`, `/map` | `nav_msgs/…` | Host intern | Odometrie, gefahrener Weg, Ultraschallkarte |

### Verzeichnisse (Teil A)

```
ros2_ws/src/shape_driver/        ROS-2-Paket (Python), gesamte Host-Seite
  shape_driver/                  Nodes:
    shape_driver_node.py           Vieleck fahren (Zeit- oder IMU-gesteuerte Drehung)
    calib_drive.py                 Kalibrierfahrten (Strecke, Drehung)
    obstacle_stop.py               Sicherheitsfilter Ultraschall + Bumper
    distance_warner.py             Buzzer-Warnstufen
    wander.py                      Zufallsfahrt mit Ausweichlogik
    odom.py                        Odometrie mit Driftkorrektur im Stillstand
    sonar_map.py                   Ultraschall-Belegungskarte
  launch/
    turtlesim_shape_launch.py      Vieleck in turtlesim (2.2)
    robot_launch.py                Sicherheitskette, optional Viereck
    explore_launch.py              komplette Erkundung inkl. RViz2
  config/                        Parameterdateien (*.yaml) und RViz2-Ansicht (explore.rviz)
firmware/
  robot_base/                    Roboter-Firmware (aktuell v7.4.1), Umgebungen serial / wifi / ota
  esp32_serial_test/, microros_int32_test/, microros_cmd_vel_test/, motor_protocol_test/,
  hw_probe/, display_test/, imu_test/      Zwischenstufen der schrittweisen Inbetriebnahme
tools/                           Auswerteskripte (Reaktionszeit, Schleifenlaufzeit, Rundumscan)
docs/                            Arbeitsprotokoll, Kartenbilder
```

### Voraussetzungen

| Komponente | Version / Hinweis |
|---|---|
| Host | Ubuntu 22.04, ROS 2 Humble (`ros-humble-desktop`, `ros-dev-tools`), Python-Paket `numpy` |
| micro-ROS-Agent | aus `micro_ros_setup` gebaut (Workspace `~/micro_ros_ws`) |
| Firmware | PlatformIO, `espressif32 @ 7.1.3`, Board `esp32dev`, `micro_ros_platformio` (Commit in `platformio.ini` festgeschrieben), U8g2 |
| Hardware | Praktikumsroboter mit ESP32, 2 Schrittmotoren mit ATtiny212-Controllern (UART), HC-SR04, 2 Bumper (JP3/JP4), OLED SSD1306 (I²C 0x3C), IMU MPU-6500 (I²C 0x68), Buzzer |

### Build

**Host (ROS 2):**
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**Firmware (ESP32):** Zugangsdaten liegen nur lokal in `include/secrets.h` (per `.gitignore` ausgeschlossen).
```bash
cd firmware/robot_base
cp include/secrets.h.example include/secrets.h     # WLAN, Agent-IP, OTA-Passwort eintragen
# erstes Aufspielen per USB:
PLATFORMIO_BUILD_FLAGS="-DDISPLAY_MODE=1 -DSTREAM_BEST_EFFORT=1" pio run -e wifi -t upload
# danach per WLAN (OTA):
export ROBOT_IP=ezmr-robot.local
export OTA_PASSWORD=$(grep "define OTA_PASSWORD" include/secrets.h | cut -d'"' -f2)
PLATFORMIO_BUILD_FLAGS="-DDISPLAY_MODE=1 -DSTREAM_BEST_EFFORT=1" pio run -e ota -t upload
```
`DISPLAY_MODE`: 0 = Display aus, 1 = eigene FreeRTOS-Task (Standard), 2 = in `loop()` (Vergleichsmessung).
`STREAM_BEST_EFFORT=1`: Sensorströme ohne Bestätigung (deutlich kürzere Schleifenlaufzeit über WLAN).

### Start

```bash
# Terminal 1 – micro-ROS-Agent
source ~/micro_ros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Terminal 2 – je nach Aufgabe:
ros2 launch shape_driver turtlesim_shape_launch.py              # 2.2: Vieleck in turtlesim
ros2 launch shape_driver robot_launch.py figur:=true            # 2.3/2.4: Viereck mit Sicherheitsfilter
ros2 launch shape_driver robot_launch.py                         # nur Sicherheitsfilter + Warner
ros2 launch shape_driver explore_launch.py duration:=180.0       # Erkundung mit Karte und RViz2
```
Bei `explore_launch.py` den Roboter vor dem Start ausrichten und danach nicht mehr berühren (Gyro-Nullpunkt und
Driftschätzung in den ersten 20 s). Strg+C beendet alles; die Karte wird nach `~/messungen/karten/` gespeichert
(`.pgm` + `.yaml` im map_server-Format, `_weg.ppm` mit gefahrenem Weg).

### Sicherheits- und Bedienhinweise

- **GPIO12 (linker Bumper, JP4) ist ein Strapping-Pin.** Vor Einschalten, EN-Reset, USB-Upload und OTA-Upload den linken Bumper abziehen, sonst startet der ESP32 nicht (`flash read err`).
- **USB-Kabel und Akku nicht gleichzeitig anschließen.** Not-Aus: Akkuschalter.
- **Tischkanten werden nicht erkannt** (Ultraschall misst nur nach vorn). Fahrten auf dem Tisch nur mit begrenzten Rändern.
- Neue Python-Dateien werden erst nach `colcon build` wirksam; im Zweifel in `install/…/site-packages/shape_driver/` prüfen.

### Bekannte Einschränkungen

- **Bodenecho:** Der Ultraschallsensor sitzt ≈ 3 cm über dem Boden. Dielenfugen quer zur Fahrtrichtung erzeugen Scheinhindernisse (0,16–0,30 m). Abhilfe: Sensor leicht nach oben neigen (nicht umgesetzt) oder auf fugenfreiem Untergrund fahren.
- **Odometrie ohne Radsensoren:** Strecke aus den gesendeten Schrittmotor-Befehlen; Schlupf oder Schrittverluste werden nicht erkannt; keine Korrektur durch die Karte (kein SLAM).
- **Ultraschallkarte:** ein einzelner, breiter Messstrahl → unscharfe Konturen statt Linien wie beim Lidar; Kegelbreite ist eine Annahme (±7,5°).
- **Pegel am ECHO-Pin:** Der HC-SR04 gibt 5 V aus, der ESP32 ist für 3,3 V ausgelegt; nicht gemessen, kein Spannungsteiler eingesetzt.
- Vergleiche zwischen Fahrvarianten beruhen jeweils auf einzelnen Fahrten.

---

## Teil B – TurtleBot 4

Arbeiten mit dem TurtleBot 4 (August 2026).

| Datei | Inhalt |
|---|---|
| `Turtlebot/turtlebot_shape.py`, `Turtlebot/mein_ausweicher/turtlebot_shape.py` | Formfahrt (Viereck oder Acht), Parameter `shape` |
| `Turtlebot/mein_ausweicher/Kreisbahn.py` | Kreisbahn über `/cmd_vel` |
| `Turtlebot/mein_ausweicher/ausweicher.py` | Hindernisausweichen mit dem Lidar (`/scan`), Parameter für Grenzabstand und Drehdauer |
| `Turtlebot/robo_test/leos_und_claras_karte.pgm/.yaml` | mit dem TurtleBot erstellte Karte |
| `Turtlebot/robo_test/bearbeitung.txt` | Mitschriften der Versuche (Akkustand, Teleoperation, Lidar-Rate, TF-Baum, autonome Navigation) |
| `Turtlebot/robo_test/…/*.png` | Screenshots |
| `Turtlebot/VID_20260818_141102.mp4` | Video (per Git LFS gespeichert, 112 MB) |

Zum Herunterladen des Videos wird Git LFS benötigt: `git lfs install && git lfs pull`.

---

## Teil C – Programmieraufgaben in C

TCP-Kommunikation zwischen Client und Server sowie erweiterte Interprozesskommunikation (System V) mit Message Queues, Shared Memory und Semaphoren; detaillierte Protokolle und Anleitungen liegen in den jeweiligen Unterordnern.

| Datei / Verzeichnis | Inhalt |
|---|---|
| `Programmieraufgaben/3/Protocol.md` & `Programmieraufgaben/4/Protocol.md` | Kommunikationsprotokolle, Nachrichtenformate und Kommandos (`ADD`, `PRIME`, `LIST`, `CLEAR`) |
| `Programmieraufgaben/3/` (`socket.c`, `client.c`, `client1.c`, `peer.c`) | Quelltexte der TCP-Socket-Implementierungen und P2P-Erweiterung |
| `Programmieraufgaben/4/` (`server_ipc.c`, `client_ipc.c`, `peer_shm.c`) | Quelltexte der System-V-IPC- und Shared-Memory-Varianten |
| `socket`, `client`, `peer`, `server_ipc`, `peer_shm` | Ausführbare Binärdateien (müssen vorab selbst kompiliert werden) |

### Übersetzen und Starten (Beispiele)

* **TCP-Sockets & P2P:**
```bash
  gcc Programmieraufgaben/3/socket.c -o socket
  gcc Programmieraufgaben/3/client.c -o client
  gcc Programmieraufgaben/3/client1.c -o client1
  gcc Programmieraufgaben/3/peer.c -lpthread -o peer
```

* **System V IPC:**
```bash
  gcc Programmieraufgaben/4/server_ipc.c -o server_ipc
  gcc Programmieraufgaben/4/client_ipc.c -o client_ipc
  gcc Programmieraufgaben/4/peer_shm.c -lpthread -o peer_shm
```
* **Starten mit ./ (Beispiele)**
```
# Terminal 1: Server starten
./server_ipc

# Terminal 2: Client starten
./client_ipc
```
Detaillierte Startanleitungen stehen in Programmieraufgaben/3/Protocol.md und Programmieraufgaben/4/Protocol.md
---

## Lizenz

Apache-2.0 (siehe `LICENSE`).

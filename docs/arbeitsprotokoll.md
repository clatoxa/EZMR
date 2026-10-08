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
| 2.2 | Eigenes ROS-2-Paket (Fahrfolge in turtlesim) | ✔ Kernaufgabe erfüllt (Paket, Node, Launchfile, Parameterdatei) |
| 2.3 | ESP32-Roboter über micro-ROS | ✔ Kernaufgabe erfüllt: `shape_driver` fährt das Viereck auf dem realen Roboter (micro-ROS über WLAN, Akku) |
| 2.4 | Sensorprojekt (Erfassung ESP32, Auswertung Host) | ⏳ in Arbeit: Ultraschall-Hinderniserkennung – Messung (10a) und Hinderniserkennung auf dem Host (10b) ✔ |
| 2.5 | TurtleBot 4 | ausgeschlossen |
| 2.6 / OTA | FreeRTOS, OTA-Updates | optional, nicht ausgewählt |

### Stufenplan

| Stufe | Inhalt | Abschnitt | Stand |
|---|---|---|---|
| 1 | ROS-2-Installation, talker/listener | 2.1 | ✔ |
| 2 | turtlesim: Nodes, Topics, Twist | 2.1 | ✔ |
| 3 | Eigenes Paket, Fahrfolge in turtlesim | 2.2 | ✔ |
| 4 | ESP32 bauen und flashen (ohne micro-ROS) | 2.3 | ✔ |
| 5 | micro-ROS-Verbindung (Int32-Publisher) | 2.3 | ✔ |
| 6 | Twist-Subscriber auf dem ESP32, Empfang ohne Motoren | 2.3 | ✔ |
| 7 | Motorprotokoll: Einzelbefehle über UART an die ATtiny212 | 2.3 | ✔ (systematisch ermittelt) |
| 8 | Differentialantrieb: Twist → Radgeschwindigkeiten | 2.3 | ✔ (aufgebockt, inkl. Timeout-Stopp) |
| 9 | `shape_driver` auf dem realen Roboter | 2.3 | ✔ (WLAN-Transport, Kalibrierung, Viereck) |
| 10 | Sensorintegration (HC-SR04) | 2.4 | ✔ 10a Messung, ✔ 10b Hinderniserkennung · ⏳ 10c Reaktionszeit, 10d Buzzer (optional) |
| 11 | Eigenes Projekt mit Host-Auswertung | 2.4 | ⏳ |
| 12 | Abgabe: Repository und Beleg | 3 | ⏳ |

---

## Systemumgebung (Host-PC)

| Komponente | Wert |
|---|---|
| Rechner | MSI MS-7C02 (B450 TOMAHAWK MAX), AMD Ryzen 5 2600 |
| Betriebssystem | Ubuntu 22.04.5 LTS (jammy), x86_64, Dual-Boot |
| Kernel | 6.8.0-138-generic (aus Kernel-Log) |
| Locale | de_DE.UTF-8 |
| ROS 2 | Humble, `ros-humble-desktop` 0.10.0-1jammy.20260908.101832 |
| Paketquelle | `ros2-apt-source` 1.3.0~jammy |
| Build-Werkzeuge | `ros-dev-tools` (colcon funktionsfähig, durch erfolgreiche Builds bestätigt) |
| ROS-Workspace | `~/robotik_praktikum/ros2_ws` (Overlay: `install/local_setup.bash`) |
| micro-ROS-Agent | Workspace `~/micro_ros_ws`, `micro_ros_setup` Branch humble |
| Entwicklungsumgebung ESP32 | VS Code (snap) mit Erweiterung PlatformIO IDE, PlatformIO Core 6.2.0 |
| Repository | `github.com/clatoxa/EZMR`, eigener Branch `leo` |
| Netzwerk Fahrbetrieb | Heim-Router; Host per Kabel (192.168.17.29), ESP32 per WLAN 2,4 GHz (192.168.17.38); Agent `udp4 --port 8888` |

**Entscheidung Arbeitsumgebung:** Auf dem Host lief zuvor Ubuntu 24.04. ROS 2 Humble wird offiziell nur für Ubuntu 22.04 als Binärpaket bereitgestellt; die Praktikumsanleitung (2.1) fordert Ubuntu 22.04. Daher wurde Ubuntu 22.04 nativ als Dual-Boot eingerichtet. Begründung gegenüber einer VM: direkter USB-Zugriff auf den ESP32 und direkte Netzwerkerreichbarkeit des micro-ROS-Agents in Abschnitt 2.3.

**Entscheidung Programmiersprachen:** Host-Nodes in Python (rclpy); ESP32-Firmware in C/C++ (Arduino-Framework + rclc/micro-ROS) gemäß Anleitung 4.1.4.

## Hardware (ESP32)

| Merkmal | Wert | Quelle |
|---|---|---|
| Chip | ESP32-D0WDQ6, Revision v1.0, Dual Core, 240 MHz | Ausgabe esptool |
| Quarz | 40 MHz | Ausgabe esptool |
| USB-Seriell-Wandler | CH340 (VID:PID 1A86:7523), Treiber `ch341-uart` | Kernel-Log, `pio device list` |
| Gerätedatei | `/dev/ttyUSB0` (Gruppe `dialout`) | `ls -l` |
| Anschluss | USB-C am Board; funktionsfähig nur mit Kabel USB-A → USB-C | Versuch |
| PlatformIO-Board | `esp32dev` („Espressif ESP32 Dev Module“) | Anleitung 4.1.4 |

**Konvention (ab 07.10.2026):** vorne = Seite mit dem Ultraschallsensor; links/rechts von hinten in Fahrtrichtung gesehen; vorwärts = Bewegung in Richtung Ultraschallsensor.

### Motorprotokoll ATtiny212 (ermittelt in Stufe 7)

| Element | Wert | Grundlage |
|---|---|---|
| Schnittstelle | UART, 9600 Baud, 8N1 (Format angenommen), nur TX an GPIO 17 | Anleitung 4.2.2/4.2.3; Versuche |
| Paket | `0xAA`, `id`, `mode`, `dir`, `param` (6 Byte) | Anleitung 4.2.2, bestätigt |
| `id` | 1 = linkes Rad, 2 = rechtes Rad, 0 = keine Reaktion | Versuche |
| `dir` | links: 1 = vorwärts, 0 = rückwärts · rechts: 0 = vorwärts, 1 = rückwärts | Versuche (Motoren spiegelbildlich) |
| `param` | 16 Bit, Big-Endian (High-Byte zuerst) | Versuch E |
| `mode 0` | feste Schrittzahl; 2048 Schritte = 1 Radumdrehung; Befehle werden gepuffert und nacheinander ausgeführt, nicht abbrechbar | Versuche A, D, F, G, G2 |
| `mode 1` | Dauerbetrieb; `param` = Wartezeit pro Schritt in ms (kleiner = schneller); neuer Befehl ersetzt den laufenden sofort | Versuche H3, H4, H6 |
| Stopp | `mode 1`, `param 0` | Versuch H5 (beide Räder) |
| Timeout im ATtiny | keiner beobachtet (> 20 s Lauf ohne neuen Befehl) | Versuch H4 |
| Versorgung | Motoren laufen auch über USB allein | Versuche mit Akku aus |

### Fahrwerk (gemessen 08.10.2026, Lineal, ca.-Werte)

| Größe | Wert |
|---|---|
| Raddurchmesser | ≈ 82 mm → Umfang ≈ 257,6 mm |
| Spurweite (Abstand der Radmitten) | ≈ 165 mm |
| Weg pro Schritt | ≈ 0,1258 mm (2048 Schritte/Umdrehung) |
| Umrechnung `mode 1` | `param` [ms] = 125,8 / v [mm/s] |
| Höchste geprüfte Radgeschwindigkeit | 62,9 mm/s (`param 2`) |
| Kalibrierte Werte (`robot.yaml`) | `linear_speed` 0,03129 m/s, `angular_speed` 0,3692 rad/s (beide Räder `param 4`) |

### Ultraschallsensor HC-SR04 (Stufe 10)

| Merkmal | Wert | Grundlage |
|---|---|---|
| Anschluss | TRIG GPIO 26, ECHO GPIO 25 | Firmware v3, funktionsfähig |
| Messtakt | 100 ms (10 Hz), Timeout 40 ms | Firmware v3; gemessen 10,000 Hz |
| Umrechnung | Abstand = Pulsdauer · 343 m/s / 2 (c fest, ≈ 20 °C) | Annahme |
| Messbereich / Öffnungswinkel | 0,02–4,0 m / 0,26 rad (≈ 15°) | typische Herstellerangaben, nicht geprüft |
| Auflösung | 1 µs ≙ 0,17 mm | `micros()` |
| Genauigkeit | ±1 cm bis 50 cm, −1,5 cm bei 100 cm | Stufe 10a |
| Pegel ECHO | 5-V-Sensor an 3,3-V-Eingang – funktioniert, formal nicht geprüft | ⚠ offen |

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

### 03.10.2026 – 2.2a Workspace und Paket

**Vorgehen:** Workspace `~/robotik_praktikum/ros2_ws/src` angelegt; Paket erzeugt mit
`ros2 pkg create --build-type ament_python --license Apache-2.0 --dependencies rclpy geometry_msgs --node-name shape_driver_node shape_driver`;
Build mit `colcon build --symlink-install`.

**Ergebnis ✔:** `ros2 run shape_driver shape_driver_node` → `Hi from shape_driver.`

**⚠ Problem:** In einem zweiten Terminal (Verzeichnis `~`) → `install/local_setup.bash: Datei oder Verzeichnis nicht gefunden`, `Package 'shape_driver' not found`.
- Ursache: relativer Pfad; Overlay nur aus dem Workspace-Verzeichnis ladbar.
- Lösung: absoluter Pfad `source ~/robotik_praktikum/ros2_ws/install/local_setup.bash`.
- Erkenntnis: Underlay (`/opt/ros/humble`) und Overlay (eigener Workspace) müssen beide geladen sein.

**Paketpflege:** `description` und `maintainer` in `package.xml` und `setup.py` ausgefüllt und abgeglichen. Dabei entstandener Syntaxfehler in `setup.py` (Vorlagenrest) mit `python3 -m py_compile` erkannt und behoben. Änderungen an `setup.py`/`package.xml` erfordern einen Neubau; `--symlink-install` gilt nur für Quelldateien.

### 03.10.2026 – 2.2b Fahr-Node `shape_driver_node`

**Entwurf:** Zeitgesteuerte Zustandsmaschine `FORWARD → TURN → … → DONE`, Timer sendet periodisch `Twist`-Befehle (wegen 1-s-Timeout von turtlesim). Phasendauern aus Geometrie: Fahrzeit = Seitenlänge / Geschwindigkeit, Drehzeit = (2π / Seitenanzahl) / Drehgeschwindigkeit. Alle Fahrwerte als ROS-Parameter (`cmd_vel_topic`, `num_sides`, `side_length`, `linear_speed`, `angular_speed`, `publish_rate`). Nach `DONE` werden fünf Nullbefehle gesendet, danach beendet sich die Node.

**⚠ Problem:** Node gab weiterhin `Hi from shape_driver.` aus → Quelldatei enthielt noch die Vorlage (Änderung nicht gespeichert). Mit `head` geprüft, Datei ersetzt.

**Ergebnis ✔:** Turtle fährt mit Standardparametern (4 Seiten, 2.0 Einheiten, 1.0 E/s, 0.5 rad/s, 10 Hz) ein Viereck und hält danach an (Screenshot 19:43).

### 03.10.2026 – Versuch: Genauigkeit der Fahrfolge in Abhängigkeit der Timerrate

**Fragestellung:** Wie groß ist die Abweichung zwischen Start- und Endpose, und wovon hängt sie ab?

**Messmethode:** `/reset`, Startpose mit `ros2 topic echo --once /turtle1/pose`, Fahrt, Endpose erneut abgefragt. Phasendauern aus den Zeitstempeln der Zustandswechsel im Log.

| | 10 Hz | 50 Hz |
|---|---|---|
| Startpose | x = 5.544, y = 5.544, θ = 0.000 | x = 5.544, y = 5.544, θ = 0.000 |
| Endpose | x = 5.444, y = 5.460, θ = 0.109 rad (6.2°) | x = 5.566, y = 5.508, θ = 0.037 rad (2.1°) |
| Abstand zum Start | ≈ 0.13 E | ≈ 0.04 E |
| Drehphase (Soll 3.142 s) | ≈ 3.20 s | ≈ 3.16 s |
| Geradeausphase (Soll 2.000 s) | ≈ 2.10 s | ≈ 2.02 s |
| Erwarteter Drehfehler (4 Ecken) | 4 × 0.0585 s × 0.5 rad/s ≈ 0.117 rad | 4 × 0.0184 s × 0.5 rad/s ≈ 0.037 rad |

**Interpretation:** Der Phasenwechsel wird erst am nächsten Timer-Tick erkannt; jede Phase verlängert sich entsprechend. Die gemessenen Endwinkel stimmen mit der Erwartung überein (10 Hz: kleine Restdifferenz, vermutlich durch diskrete Simulationsschritte – nicht geprüft). Der Fehler ist systematisch und hängt von der Lage der Sollzeit im Timer-Raster ab; fällt die Sollzeit genau auf einen Tick, verlängert Timer-Jitter die Phase um einen ganzen Takt. Höhere Rate verringert den Fehler (hier etwa Faktor 3).

**Offen ⏳:** Belastbarkeit hoher Befehlsraten über micro-ROS/ESP32 (Abschnitt 2.3) ungeprüft. Mögliche Verbesserung ohne höhere Rate: Restzeit-Korrektur im letzten Tick einer Phase (nicht umgesetzt).

### 03.10.2026 – 2.2 Launchfile und Parameterdatei

**Neu:**
- `config/turtlesim.yaml`: Fahrparameter für die Simulation (`publish_rate` 50 Hz)
- `launch/turtlesim_shape_launch.py`: startet `turtlesim_node` und – um 2 s verzögert (`TimerAction`) – `shape_driver_node` mit der Parameterdatei; Argument `params_file` zur Auswahl einer anderen YAML-Datei
- `setup.py`: `data_files` um `launch/` und `config/` ergänzt
- `package.xml`: `exec_depend` für `launch`, `launch_ros`, `ament_index_python`, `turtlesim`

**Ergebnis ✔:** `ros2 launch shape_driver turtlesim_shape_launch.py` startet beide Nodes; Fahrfolge läuft vollständig durch, `shape_driver_node` beendet sich sauber (`process has finished cleanly`).

**Nachweis, dass die YAML-Datei geladen wurde:** Drehphasen dauerten ≈ 3.16 s – das entspricht dem 50-Hz-Raster aus der YAML-Datei, nicht dem Code-Standardwert von 10 Hz (≈ 3.20 s).

**Zweck:** Trennung von Code und Parametern; für Abschnitt 2.3 wird eine eigene Parameterdatei für den Roboter ergänzt, ohne den Code zu ändern.

### 04.10.2026 – Versionierung

**Vorgehen:** SSH-Schlüssel (ed25519) für den Host-PC erzeugt und bei GitHub hinterlegt. Gemeinsames Repository `github.com/clatoxa/EZMR` (bestehende Ordner `Programmieraufgaben/`, `Turtlebot/`, keine Namenskonflikte). Eigener Branch `leo` auf Basis von `origin/main` angelegt.

**⚠ Zwischenfall:** `git add` brach ab, weil `.gitignore` und `README.md` noch nicht existierten (Download fehlgeschlagen). Dadurch wurde nichts vorgemerkt – ohne `.gitignore` wären sonst `build/`, `install/`, `log/` mit hinzugefügt worden. Dateien danach direkt im Terminal erzeugt.

**Ergebnis ✔:** Commit `1bed40a` (16 Dateien: Paket `shape_driver` inkl. Launchfile und Parameterdatei, Arbeitsprotokoll, README, `.gitignore`, LICENSE); Push auf `origin/leo`. Generierte colcon-Ordner nicht enthalten.

### 04.10.2026 – 2.3 Stufe 4: USB-Verbindung zum ESP32

**Sicherheit:** Roboter aufgebockt (Räder frei), Akku aus, Versorgung des ESP32 nur über USB – unbekannte Vorgänger-Firmware hätte Motoren ansteuern können.

**⚠ Problem 1 – Gerät wird nicht angemeldet:** Mit Kabel USB-C ↔ USB-C an einem hinteren Mainboard-Port: `device descriptor read/64, error -71`, `Device not responding to setup address`, `unable to enumerate USB device`. Board erhielt Strom (rote LED), Datenübertragung gestört.
- Lösung: Kabel USB-A → USB-C. Danach Treiber `ch341-uart` geladen (CH340).

**⚠ Problem 2 – Gerät wird sofort wieder getrennt:** Log: `ch341-uart converter now attached to ttyUSB0`, direkt danach `usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1` und `now disconnected from ttyUSB0`.
- Ursache: Paket `brltty` (Braillezeilen-Unterstützung) belegt den CH340.
- Lösung: `sudo apt remove brltty`.
- Ergebnis ✔: `/dev/ttyUSB0` bleibt bestehen (`crw-rw---- root dialout`).

**⚠ Problem 3 – Zugriffsrechte:** Benutzer mit `sudo usermod -a -G dialout $USER` zur Gruppe `dialout` hinzugefügt. Nach Ab- und Anmelden zeigte `groups` die Gruppe weiterhin nicht (`groups $USER` dagegen schon). Erst nach einem Neustart aktiv.

**⚠ Problem 4 – veraltetes PlatformIO:** Im Suchpfad lag `pio` 4.3.4 aus dem Ubuntu-Paket `platformio`; `micro_ros_platformio` erfordert PlatformIO Core ≥ 6.1.0. Die VS-Code-Erweiterung war noch nicht installiert (`~/.platformio/penv` fehlte).
- Lösung: `sudo apt remove platformio`; `python3-venv`, `git`, `cmake`, `python3-pip` installiert; alter Ordner nach `~/.platformio_alt_4.3.4` verschoben; Erweiterung mit `code --install-extension platformio.platformio-ide` installiert → PlatformIO Core 6.2.0.
- Shell-Befehle per Symlink nach `~/.local/bin` (PlatformIO-Doku „Install Shell Commands“). In neuen Terminals erst nach erneuter Anmeldung verfügbar, da `~/.local/bin` nach dem Login angelegt wurde.
- PlatformIO-udev-Regeln installiert (`/etc/udev/rules.d/99-platformio-udev.rules`).

### 04.10.2026 – 2.3 Stufe 4: Testfirmware

**Vorabtest (vorhandene Firmware):** `pio device monitor -p /dev/ttyUSB0 -b 115200`, Reset über EN. Ausgabe zeigt die vollständige Bootsequenz des ESP32-Boot-ROMs (`POWERON_RESET`, `SPI_FAST_FLASH_BOOT`, `load …`, `entry 0x400805e4`) → ESP32 startet, keine Neustartschleife. Vorhandene Firmware sendet danach nichts (bei 115200 Baud).
- ⚠ Beobachtung: Boot-ROM-Ausgabe teilweise verfälscht (einzelne Zeichen).

**Projekt:** `firmware/esp32_serial_test` (`pio project init --board esp32dev --project-option "framework=arduino"`, `monitor_speed = 115200`). Programm gibt nur einen Zähler aus, steuert keine Pins an.

**Ergebnis ✔:**
- Upload mit 460800 Baud, alle Segmente `Hash of data verified`, `[SUCCESS]`.
- esptool: Chip ESP32-D0WDQ6 (revision v1.0), Dual Core, 240 MHz, Quarz 40 MHz.
- Monitor: `ESP32 Testfirmware gestartet`, danach `Zaehler: 0, 1, 2, …` im 1-s-Takt, fehlerfrei.

**Interpretation:** Programmausgabe und Upload fehlerfrei → kein grundsätzliches Signalproblem. Die Zeichenfehler betrafen nur die Boot-ROM-Ausgabe nach dem Reset; Ursache ungeklärt, derzeit ohne Auswirkung.

### 04.10.2026 – 2.3 Stufe 5a: micro-ROS-Agent

**Vorgehen** (Anleitung 4.1.3, abgeglichen mit README `micro_ros_setup`, Branch humble):
1. rosdep geprüft bzw. initialisiert
2. Workspace `~/micro_ros_ws` (außerhalb des Repos), `micro_ros_setup` Branch humble geklont
3. `rosdep update`, `rosdep install --from-paths src --ignore-src -y`, `colcon build`
4. `ros2 run micro_ros_setup create_agent_ws.sh`, `ros2 run micro_ros_setup build_agent.sh`

**Abweichung von der Anleitung:** Die Anleitung installiert `python3-rosdep2` (Ubuntu-Paket). Die offizielle README nennt `python3-rosdep` (ROS-Paket); beide Pakete stehen im Konflikt. Es wurde `python3-rosdep` verwendet.

**Ergebnis ✔:** `ros2 pkg executables micro_ros_agent` → `micro_ros_agent micro_ros_agent`.

### 04.10.2026 – 2.3 Stufe 5b: micro-ROS-Verbindungstest (Int32)

**Projekt:** `firmware/microros_int32_test`, Code aus Anleitung 4.1.4 (inhaltlich unverändert): Node `micro_ros_platformio_node`, Publisher `std_msgs/Int32` auf `micro_ros_platformio_node_publisher`, rclc-Timer 1000 ms; `loop()`: `delay(100)` + `rclc_executor_spin_some(…, 100 ms)`.

**Abweichung von der Anleitung:** In `platformio.ini` ausdrücklich gesetzt:
```ini
board_microros_distro = humble
board_microros_transport = serial
```
Grund: Laut aktueller README von `micro_ros_platformio` ist die Standard-Distribution `kilted`; ohne Angabe würde die Firmware nicht zum Humble-Agent passen.

**Start:** Terminal 1: `source ~/micro_ros_ws/install/local_setup.bash`, `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0`; danach ESP32 über EN neu gestartet.

**Ergebnis ✔:**
- `ros2 node list` → `/micro_ros_platformio_node`
- `ros2 topic list` → `/micro_ros_platformio_node_publisher`
- `ros2 topic echo /micro_ros_platformio_node_publisher` → `data: 9, 10, 11, 12, …`
- Echo begann bei 9, da ein Subscriber nur Nachrichten nach seinem Start erhält.

**Hinweise für die Weiterarbeit:**
- `Serial` ist micro-ROS-Transport → keine `Serial.print`-Debugausgaben; serieller Monitor und Agent nicht gleichzeitig (Port belegt).
- Agent vor dem ESP32-Reset starten; ohne Agent bleibt der Beispielcode in `error_loop()` hängen.

### 04.10.2026 – Messung: Periode des Int32-Publishers

**Messmethode:** `ros2 topic hz /micro_ros_platformio_node_publisher` (Ankunftszeiten auf dem Host).

| Größe | Wert |
|---|---|
| Sollperiode | 1,000 s |
| Mittlere Rate | 0,990 Hz |
| Minimale Periode | 0,907 s |
| Maximale Periode | 1,093 s |
| Standardabweichung | 0,09 s |
| Stichprobe | 9 Intervalle |

**Interpretation (Hypothese, nicht gezielt geprüft):** Der Executor prüft den Timer nur im ~100-ms-Raster von `loop()`. Ein verspäteter Aufruf verlängert ein Intervall; da der Timer seinen Takt hält, verkürzt sich das folgende. Zusätzlich Übertragungslatenz (seriell → Agent → DDS).

**Folgerung:** Für den Twist-Subscriber (Stufe 6) `delay()` in `loop()` deutlich verkürzen, da Befehle mit z. B. 50 Hz ankommen sollen.

**Offen ⏳:** Gegenprobe mit `delay(10)` und längere Messung (≥ 60 Intervalle).

### 04.10.2026 – Versionen (funktionierende Kombination)

| Komponente | Version |
|---|---|
| PlatformIO Core | 6.2.0 |
| Plattform `espressif32` | 7.1.3 |
| `framework-arduinoespressif32` | 4.20017.260907 (sha dcc1105b) |
| `toolchain-xtensa-esp32` | 8.4.0+2021r2-patch5 |
| `tool-esptoolpy` | 2.41100.260830 (esptool.py v4.11.0) |
| `micro_ros_platformio` | Commit cfee17f |
| micro-ROS-Distribution | humble |
| micro-ROS-Agent | `micro_ros_setup`, Branch humble |

**Entscheidung:** Ab Stufe 6 Plattform- und Bibliotheksversion in `platformio.ini` festschreiben (`platform = espressif32 @ 7.1.3`, `…/micro_ros_platformio#cfee17f`), damit spätere Builds reproduzierbar bleiben.

### 07.10.2026 – 2.3 Stufe 6: Twist-Subscriber (ohne Motoren)

**Ziel:** Nachweis, dass der ESP32 Bewegungsbefehle (`geometry_msgs/msg/Twist`) auf `/cmd_vel` empfängt – ohne Motoransteuerung.

**Problem:** `Serial` ist als micro-ROS-Transport belegt, empfangene Werte können nicht direkt ausgegeben werden.
**Lösung:** Jede empfangene Nachricht wird unverändert auf ein zweites Topic `/cmd_vel_echo` zurückgesendet.

```text
ros2 topic pub /cmd_vel → Agent → ESP32 (Subscriber) → Publisher /cmd_vel_echo → Agent → ros2 topic echo
```

**Projekt:** `firmware/microros_cmd_vel_test`
- Node `esp32_cmd_vel_test`; Subscriber `cmd_vel`, Publisher `cmd_vel_echo` (beide `geometry_msgs/Twist`)
- Executor mit 1 Handle (Subscription, `ON_NEW_DATA`); Callback publiziert die empfangene Nachricht
- `loop()`: **ohne `delay()`**, `rclc_executor_spin_some(…, 10 ms)` (Folgerung aus der Periodenmessung in Stufe 5)
- Versionen festgeschrieben: `platform = espressif32 @ 7.1.3`, `micro_ros_platformio#cfee17f` – bestätigt durch `pio pkg list` (`espressif32 @ 7.1.3`, `micro_ros_platformio @ 0.0.1+sha.cfee17f`)
- Firmware nutzt weder GPIO 17 noch UART2; Akku aus.

**⚠ Zwischenfall:** Nach dem Start des Agents keine Verbindung (`ros2 node list` ohne ESP32-Node, Agent-Log nur `running…`).
- Ursache: ESP32 startete direkt nach dem Flashen, als noch kein Agent lief → `error_loop()`.
- Lösung: EN am ESP32 nach dem Agent-Start.
- Agent-Log danach: `session established`, `participant created`, `topic created`, `subscriber`/`datareader created`, `publisher`/`datawriter created` – entspricht den Initialisierungsschritten in `setup()`.

**Test 6a – Empfang:** `ros2 topic pub --rate 2 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.5}}"`
**Ergebnis ✔:** `ros2 node list` → `/esp32_cmd_vel_test`; `ros2 topic echo /cmd_vel_echo` → `linear.x: 0.1`, `angular.z: 0.5` (identisch mit den gesendeten Werten).

**Test 6b – Durchsatz bei 50 Hz:** `ros2 topic pub --rate 50 /cmd_vel …`; Messung mit `ros2 topic hz /cmd_vel_echo` (Rundreise Host → ESP32 → Host).

| Größe | Stufe 5 (Int32, `delay(100)`) | Stufe 6b (Echo, ohne `delay`) |
|---|---|---|
| Sollperiode | 1000 ms | 20 ms |
| Mittlere Rate | 0,990 Hz | 50,000 Hz |
| min / max | 907 / 1093 ms | 19 / 21 ms |
| Standardabweichung | ≈ 90 ms | ≈ 0,17 ms |
| Stichprobe | 9 Intervalle | 706 Intervalle |

**Interpretation:** Die Verbindung über USB-Seriell trägt 50 Hz stabil; Empfangsrate = Senderate, kein Verlust erkennbar (von `ros2 topic hz` nicht ausdrücklich gezählt, daher nicht streng geprüft). Der Jitter sank von ±90 ms auf ±1 ms – das stützt die Hypothese aus Stufe 5, dass `delay(100)` in `loop()` die Schwankungen verursachte (Vergleich nicht streng, da Aufbau verschieden: Timer vs. Echo).

**Folgerung:** `shape_driver` kann auch auf dem Roboter mit 50 Hz senden.

**Test 6c – eigene Node → ESP32:** `ros2 run shape_driver shape_driver_node --ros-args -p cmd_vel_topic:=/cmd_vel -p publish_rate:=10.0`
**Ergebnis ✔:** `/cmd_vel_echo` zeigt abwechselnd `linear.x: 1.0` (FORWARD) und `angular.z: 0.5` (TURN), am Ende Nullbefehle → Kette `shape_driver` → `/cmd_vel` → Agent → ESP32 nachgewiesen. (Werte noch aus der turtlesim-Konfiguration; Roboterwerte folgen in Stufe 9.)

### 07.10.2026 – Vorbereitung Stufe 7: Motorprotokoll

**Ausgangslage:** Die Anleitung (4.2.2, 4.2.3) dokumentiert:
- je ein ATtiny212 pro Motor erzeugt die Schrittfolgen für 28BYJ-48 + ULN2003
- ESP32 sendet über den zweiten Hardware-UART, nur TX an GPIO 17
- Paketformat: `START_BYTE 0xAA` (uint8), `id` (uint8), `mode` (uint8), `dir` (uint8), `param` (uint16)
- `mode 0` = feste Schrittanzahl, `mode 1` = Dauerbetrieb; `param` = Schrittanzahl bzw. Drehgeschwindigkeit

**Nicht dokumentiert** (auch kein Verweis auf ATtiny-Quellcode im Quellenverzeichnis): Baudrate/UART-Format, Motor-IDs, Codierung von `dir`, Byte-Reihenfolge von `param`, Einheit/Wertebereich der Geschwindigkeit, Stoppbefehl, Versorgung der Motoren.

**Hypothese Versorgung:** Die Motoren benötigen vermutlich den eingeschalteten 18650-Akku (Anleitung Tabelle 1 / Abbildung 2: Akku mit Lademodul und An/Aus-Schalter, „Versorgung bis 2 A“). Nicht dokumentiert, in Stufe 7 zu prüfen. Hinweis der Anleitung: beim Einlegen des Akkus Polarität beachten.

**Entscheidung:** Protokolldetails werden systematisch durch Versuche ermittelt; parallel Anfrage an die Betreuung (ATtiny-Quellcode oder funktionierendes Beispiel).

**Geplantes Vorgehen (durchgeführt am 07.10.2026, siehe folgende Einträge):**
- Eigene Testfirmware ohne micro-ROS: Befehle über den seriellen Monitor eingeben, ESP32 sendet das Paket byteweise (explizite Serialisierung) über GPIO 17 und zeigt die gesendeten Bytes an.
- Sicherheit: Roboter aufgebockt, Räder frei; zunächst nur `mode 0` (begrenzte Bewegung, endet selbstständig); Not-Aus über den Akkuschalter.
- `param = 257 (0x0101)` für erste Versuche, da in beiden Byte-Reihenfolgen identisch (umgesetzt mit 1028 = 0x0404).
- Reihenfolge: (1) Baudrate × ID bis zur ersten Bewegung, (2) ID ↔ linkes/rechtes Rad, (3) `dir`, (4) Byte-Reihenfolge (z. B. 0x0102 vs. 0x0201), (5) `mode 1`: Geschwindigkeit und Stopp.
- Annahmen: UART-Format 8N1; gemeinsame Masse von ESP32 und ATtiny.

**Sicherheitsstufen für Bewegungstests:**

| Phase | Aufbau |
|---|---|
| Stufe 7 | aufgebockt, Räder frei, nicht an der Tischkante |
| Stufe 8 | aufgebockt; Stopp und Timeout zuerst nachweisen |
| Stufe 9 | Bodenfahrt erst nach nachgewiesenem Timeout-Stopp; ausreichend langes Kabel oder WLAN-Transport |

### 07.10.2026 – 2.3 Stufe 7a: Testkonsole Motorprotokoll

**Projekt:** `firmware/motor_protocol_test` (ohne micro-ROS; `Serial` = Eingabekonsole, `monitor_filters = send_on_enter`, `monitor_echo = yes`). UART2 an GPIO 17 (TX), RX-Pin 16 nur formal; Start mit 9600 Baud. Explizite byteweise Serialisierung, Byte-Reihenfolge von `param` umschaltbar, Senden nur auf Befehl. Befehle: `send`, `baud`, `order le|be`, `raw`, `status`, `help` (später `stop`).

**Trockentest (Akku aus):** TX-Bytes entsprechen den Befehlen (z. B. `send 1 0 0 1028` → `AA 01 00 00 04 04`; `order be` + `send 1 0 0 258` → `… 01 02`).

**⚠ Auffälligkeit 1:** `order le` + `send 1 0 0 258` ergab `… 03 01` (259).
- Prüfung: Firmware meldet empfangene Zeile zurück (`RX: '…'`); 16 Wiederholungen fehlerfrei.
- Zunächst als Tippfehler gewertet.

**⚠ Auffälligkeit 2:** `send 1 0 0 1028` wurde abgelehnt, obwohl `RX` scheinbar exakt den richtigen Text zeigte.
- Ursache (wahrscheinlich): Backspace-Zeichen wurden in den Zeilenpuffer übernommen; das Terminal führte sie bei der `RX`-Ausgabe aus, sodass die Zeile korrekt aussah. Erklärt vermutlich auch Auffälligkeit 1.
- Risiko: falscher `param` bei korrekt aussehender Anzeige.
- Lösung: Backspace wird im Puffer verarbeitet; Zeilen mit Steuer- oder Nicht-ASCII-Zeichen werden verworfen.
- Test ✔: Eingabe `1029` → Backspace → `8` ergibt `RX: 'send 1 0 0 1028'`, `TX … 04 04`.

### 07.10.2026 – 2.3 Stufe 7b: Ermittlung des Motorprotokolls

**Aufbau:** Roboter aufgebockt, Räder frei, Klebeband-Markierung an den Rädern; Akku aus (Versorgung über USB). Beobachtung per Sichtprüfung, Zeitmessung per Handstoppuhr.

**Hinweis Seitenzuordnung:** Die Versuche A–D wurden zunächst ohne festgelegte Blickrichtung protokolliert (vermutlich Blick von vorne) – links/rechts waren dadurch vertauscht. Nach Festlegung der Konvention (vorne = Ultraschallsensor, Blick von hinten) wurde kontrolliert: `send 1 0 1 1028` → links vorwärts; `send 2 0 0 1028` → rechts vorwärts. Die folgenden Angaben sind korrigiert.

| Versuch | Befehl(e) | Beobachtung | Ergebnis |
|---|---|---|---|
| A | `send 1 0 0 1028` @ 9600, mehrfach | ein Rad (links) dreht, jedes Mal gleich weit, endet selbstständig | Baudrate 9600; mode 0 wiederholbar |
| B | `send 0/2 0 0 1028` | ID 0: keine Reaktion; ID 2: anderes Rad (rechts) | ID 1 = links, ID 2 = rechts |
| C | `dir 1` statt `dir 0` | entgegengesetzte Drehrichtung | `dir` bestimmt Richtung |
| D | `send 1 0 0 2056` (0x0808) | doppelter Winkel gegenüber 1028 | Winkel ∝ Schrittzahl |
| E | Referenz `1028`; `order le`+`260` (`04 01`); `order be`+`260` (`01 04`) | Referenz: ½ Umdrehung; `04 01`: ½ Umdrehung; `01 04`: ≈ ¼ der Referenz | `param` Big-Endian |
| F | `send 1 0 0 2048` (be) | genau eine Umdrehung | 2048 Schritte/Umdrehung |
| G | `send 1 0 0 4112` (≈ 2 U.), währenddessen `send 1 0 0 0` | beide Umdrehungen laufen zu Ende | mode 0 nicht abbrechbar |
| G2 | `send 1 0 0 4112`, währenddessen `send 1 0 0 1028` | danach weitere ½ Umdrehung | mode-0-Befehle werden gepuffert |
| Richtung | `dir 0`/`dir 1` je Rad, Konvention s. o. | links: dir 1 vorwärts; rechts: dir 0 vorwärts | Motoren spiegelbildlich |
| H1/H3 | `send 1 1 1 257`, `… 10`, `… 5`, `… 2` | 257: LEDs der ULN2003-Platine blinken sichtbar nacheinander; 10: sehr schnelles Blinken; 5: langsame, 2: schnellere Drehung | kleinerer `param` = schneller |
| H5 | `send 1 1 1 5`, dann `send 1 1 1 0`; rechts: `send 2 1 0 5`, dann `send 2 1 0 0` | stoppt jeweils sofort | Stopp = mode 1, param 0 |
| H6 | `send 1 1 1 5`, währenddessen `send 1 1 1 2` | sofortiger Wechsel auf höhere Drehzahl | mode-1-Befehle ersetzen sofort |

**Versuch H4 – Zeitmessung `mode 1` (linkes Rad, `dir 1`, Handstoppuhr):**

| `param` | Messung | s/Umdrehung | erwartet (2048 × param ms) | Abweichung | Schrittperiode |
|---|---|---|---|---|---|
| 2 | 3 U. in 12,42 s | 4,14 | 4,10 | +1,1 % | 2,02 ms |
| 5 | 2 U. in 20,76 s | 10,38 | 10,24 | +1,4 % | 5,07 ms |
| 10 | 1 U. in 20,57 s | 20,57 | 20,48 | +0,4 % | 10,04 ms |

**Ergebnis:** In `mode 1` ist `param` die Wartezeit pro Schritt in ms (Schrittrate = 1000 / `param`). Abweichungen < 1,5 %, durchweg positiv – vermutlich Handstoppung und ggf. Zeitbedarf pro Schritt im ATtiny (nicht getrennt geprüft). Höchste geprüfte Drehzahl: `param 2` ≈ 0,24 U/s; `param 1` ungetestet. Mögliche Drehzahlen sind grob gestuft (1000, 500, 333, 250, 200 … Schritte/s).

**Beobachtung Timeout:** Im Dauerbetrieb lief der Motor > 20 s ohne neuen Befehl → im ATtiny kein (kürzerer) Timeout; ein Stopp bei Verbindungsverlust muss vom ESP32 ausgehen.

**⚠ Hypothese widerlegt – Versorgung:** Alle Versuche liefen bei ausgeschaltetem Akku. Die Motoren werden auch über USB versorgt. Folgerung: Not-Aus = USB-Kabel trennen (Akkuschalter allein unwirksam, solange USB steckt). Auf Brownout-Neustarts achten (erneute Startmeldung), besonders bei zwei laufenden Motoren – bisher nicht beobachtet.

**⚠ Zwischenfall – lang anhaltende Bewegung:** Tippfehler `send 1 0 0 2026` (statt 2056) → Rad dreht lang anhaltend; gestoppt durch Trennen des USB-Kabels (Akku aus). Erklärung: 2026 = 0x07EA wurde mit Low-Byte zuerst gesendet (`EA 07`), der ATtiny liest Big-Endian → 0xEA07 = 59.911 Schritte (≈ 29 Umdrehungen). Der Zwischenfall lieferte den ersten Hinweis auf die Byte-Reihenfolge (später in Versuch E bestätigt).

**Gegenmaßnahmen in der Konsole:**
- Grenze 4200 für `param`, geprüft in **beiden** Byte-Lesarten (verhindert Fehler dieser Art auch bei falscher Reihenfolge)
- zunächst nur `mode 0`, nach Ermittlung des Stopps `mode 0/1`; `dir` nur 0/1
- `raw` nur als vollständiges, geprüftes 6-Byte-Paket
- Prüfung ✔: `send 1 0 0 2026`, `send 1 1 0 100` (vor Freigabe), `send 1 0 2 100`, `raw AA 01 00 00 EA 07`, `raw AA 01 00 00 04` → jeweils abgelehnt
- Standard der Byte-Reihenfolge auf Big-Endian umgestellt; Befehl `stop` (beide Räder mode 1, param 0) ergänzt; Kopfkommentar mit allen ermittelten Werten

**Bewertung:** Alle Werte beruhen auf Versuchen mit Sichtprüfung bzw. Handstoppung an frei drehenden Rädern. Unter Last (Bodenfahrt) können Schrittverluste auftreten; das ist in Stufe 8/9 zu prüfen. Die Anfrage an die Betreuung zum ATtiny-Quellcode bleibt zum Abgleich sinnvoll.

**Folgerungen für Stufe 8:**
1. Fahrbetrieb ausschließlich mit `mode 1` (sofortige Änderung und Stopp möglich).
2. Umrechnung `Twist` → Radgeschwindigkeit → Raddrehzahl → `param` (ms/Schritt); dafür Raddurchmesser und Spurweite messen.
3. Grobe Drehzahlstufen → sinnvolles Runden; Mindestwert für `param` festlegen.
4. Richtungszuordnung je Rad (links vorwärts `dir 1`, rechts vorwärts `dir 0`).
5. Timeout im ESP32: ohne neuen `Twist` (z. B. 0,5 s) beide Räder stoppen.

### 08.10.2026 – 2.3 Stufe 8: Differentialantrieb

**Messung:** Raddurchmesser ≈ 82 mm, Spurweite ≈ 165 mm (Lineal). Unsicherheit ±1 mm beim Durchmesser ≈ ±1,2 % Streckenfehler → Kalibrierfahrt in Stufe 9 vorgesehen.

**Rechnung:**
```text
Umfang       U = π · 82 mm        ≈ 257,6 mm
Weg/Schritt  s = U / 2048          ≈ 0,1258 mm
param [ms]     = 1000 · s / v      = 125,8 / v [mm/s]
v_links  = v − ω · L/2,   v_rechts = v + ω · L/2   (L = 0,165 m, ω > 0 = gegen den Uhrzeigersinn)
```

| `param` | Schritte/s | Radgeschwindigkeit |
|---|---|---|
| 1 | 1000 | 126 mm/s (ungetestet, nicht verwendet) |
| 2 | 500 | 62,9 mm/s |
| 3 | 333 | 41,9 mm/s |
| 4 | 250 | 31,5 mm/s |
| 5 | 200 | 25,2 mm/s |
| 10 | 100 | 12,6 mm/s |

**Folgerung:** Höchstgeschwindigkeit ≈ 6 cm/s; Drehen auf der Stelle max. ≈ 0,76 rad/s. Geschwindigkeiten sind grob gestuft (z. B. 50 mm/s nicht darstellbar: 63 oder 42 mm/s) → für genaue Fahrfiguren Geschwindigkeiten wählen, die genau einer `param`-Stufe entsprechen.

**Firmware:** `firmware/robot_base` (Versionen wie Stufe 6 festgeschrieben)
- Node `robot_base`; Subscriber `/cmd_vel` (Twist); Kontrolltopic `/wheel_cmd` (`geometry_msgs/Vector3`: x/y = vorzeichenbehafteter `param` links/rechts, + vorwärts, 0 Stopp; z = 1 bei Timeout)
- Callback speichert nur `v`, `ω` und Empfangszeit; UART-Ausgabe im Regelschritt (alle 50 ms, `millis()`-basiert in `loop()`)
- Timeout 500 ms ohne Twist → beide Räder Stopp (ATtiny hat keinen eigenen Timeout)
- Stopp beim Start (vor micro-ROS-Initialisierung) und in `error_loop()`
- Proportionale Sättigung auf v_max = 62,9 mm/s (Kurvenform bleibt erhalten)
- Rundung auf ganzzahlige `param`, Bereich 2…100; langsamer als `param 100` (< 1,3 mm/s) → Stillstand
- UART-Paket nur bei Änderung senden (ein Paket ≈ 6 ms bei 9600 Baud)
- Bekannte Einschränkung: keine automatische Wiederverbindung zum Agent (Reset per EN nötig)

**Tests** (aufgebockt, Agent über USB, `ros2 topic echo /wheel_cmd`):

| Test | Befehl | Erwartung | `/wheel_cmd` | Räder |
|---|---|---|---|---|
| T1 | kein Befehl | Stillstand, z 1 | – | stehen ✔ |
| T2 | `linear.x 0.03` (10 Hz) | 125,8/30 ≈ 4,2 → param 4/4 | – | beide vorwärts ✔ |
| T3 | T2 beendet | Stopp ≤ 0,5 s | – | stoppen ✔ |
| T4 | `angular.z 0.5` | ±41,25 mm/s → param 3; links rückwärts, rechts vorwärts | x −3, y 3, z 0 ✔ | wie erwartet ✔ |
| T5 | `linear.x 0.2` | gesättigt → param 2/2 | x 2, y 2, z 0 ✔ | schneller als T2 ✔ |
| T6 | T2 läuft, Agent beendet | Stopp ≤ 0,5 s | – | stoppen ✔ |

**Ergebnis ✔:** Kinematik, Richtungszuordnung, Rundung, Sättigung und Timeout-Stopp (bei Senderausfall und bei Verbindungsabbruch) entsprechen der Vorhersage. Voraussetzung für die Bodenfahrt erfüllt.

### 08.10.2026 – Vorbereitung Stufe 9

**Problem:** Verfügbares datenfähiges USB-Kabel nur ≈ 1 m – Roboter würde bei Bodenfahrt am Kabel ziehen (Einfluss auf die Fahrfigur, Gefahr für die USB-Buchse).

**Entscheidung:** micro-ROS-Transport auf WLAN (UDP) umstellen (Anleitung 4.1.4: Agent mit `udp4 --port 8888`; micro_ros_platformio: `board_microros_transport = wifi`, `set_microros_wifi_transports(ssid, psk, agent_ip, agent_port)`). Zugangsdaten nicht im öffentlichen Repository (`secrets.h`, per `.gitignore` ausgeschlossen).

**Geplante Fahrparameter (`config/robot.yaml`, Ausgangswerte vor der Kalibrierung):** `linear_speed` 0,03145 m/s und `angular_speed` 0,3812 rad/s (beide Räder genau `param 4`), `side_length` 0,30 m, `publish_rate` 20 Hz. Reihenfolge: Geradeausfahrt 1 m (Kalibrierung) → Drehung 360° → Viereck.

### 08.10.2026 – 2.3 Stufe 9a: micro-ROS über WLAN

**Umsetzung `firmware/robot_base`:**
- `platformio.ini` mit zwei Umgebungen bei identischem Code: `env:serial` (`board_microros_transport = serial`) und `env:wifi` (`= wifi`); Auswahl im Code über `MICRO_ROS_TRANSPORT_ARDUINO_SERIAL` / `_WIFI`
- WLAN: `set_microros_wifi_transports(ssid, psk, agent_ip, 8888)` (laut Bibliotheksquelle blockierend bis zur WLAN-Verbindung → Motorstopp vorher); `WiFi.setSleep(false)` (Annahme: geringere Latenz)
- Im WLAN-Modus ist `Serial` frei → Statusausgaben (Verbindung, IP, Timeout, Fehlerzeile in `error_loop`)
- Zugangsdaten in `include/secrets.h` (lokal), per `.gitignore` ausgeschlossen – geprüft mit `git check-ignore -v` (`.gitignore:17 **/secrets.h`); im Repository nur `include/secrets.h.example`
- Agent: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`; `ufw` inaktiv

**⚠ Zwischenfälle:**
- Falsche `main.cpp` geflasht (gleichnamiger Download `main(1).cpp` → Testkonsole statt `robot_base`); erkannt an der Monitorausgabe. Lösung: eindeutige Dateinamen (`robot_base_main.cpp`).
- Upload-Fehler „port is busy“: offener serieller Monitor (`fuser -v /dev/ttyUSB0` → `pio`).
- Keine Verbindung nach Neustart des ESP32 ohne laufenden Agent bzw. nach Beenden des Agents → Reihenfolge: Agent starten, dann EN. Vor jeder Fahrt `ros2 node list` prüfen.

**Ergebnis Verbindung ✔:** `session established` von 192.168.17.38; Node `/robot_base`.

**Befund Drift im Regeltakt:** `/wheel_cmd` kam mit 18,6 Hz statt 20 Hz (23–79 ms). Ursache: `lastControlMs = now` – Verspätungen der Schleife summieren sich. Korrektur: `lastControlMs += CONTROL_PERIOD_MS` (Soll-Takt) mit Schutz gegen Nachholen nach langer Blockade.

| | vorher | nachher |
|---|---|---|
| Mittlere Rate | 18,6 Hz | 20,0 Hz |
| min / max | 23 / 79 ms | 42 / 59 ms |
| Standardabweichung | ≈ 3,3 ms | ≈ 4,2 ms |
| Stichprobe | 228 | 165 |

Interpretation: Drift (systematisch, summiert sich) beseitigt; verbleibender Jitter ±8 ms (Schleifenlaufzeit, WLAN), der sich im Mittel ausgleicht.

**⚠ Versorgung:** Im WLAN-Modus mit USB-Versorgung (Akku aus) drehten die Motoren nicht, obwohl `/wheel_cmd` korrekt war (x 4, y 4, z 0) und alle LEDs der ULN2003-Platinen leuchteten (bei 250 Schritten/s als Dauerleuchten wahrgenommen). Mit Akku (USB getrennt) drehen beide Räder. Wahrscheinliche Ursache: USB-Versorgung reicht nicht für ESP32 + WLAN + Motoren (nicht gemessen).
- Regel: Flashen/Monitor über USB (Akku aus); Fahrbetrieb über Akku (USB getrennt); nie beides gleichzeitig (Verhalten der Platine unbekannt).
- Not-Aus im Akkubetrieb: Akkuschalter ✔ (sofortiger Stillstand).

**Verbesserung Sicherheit:** Der aktuelle Motorbefehl (auch Stopp) wird zusätzlich jede Sekunde wiederholt (`RESEND_PERIOD_MS = 1000`), da der UART zu den ATtinys keine Bestätigung kennt (Schutz gegen verlorene Stopp-Befehle).

**Tests mit Akku über WLAN (aufgebockt):** T2 beide vorwärts ✔; T3 schneller Stopp ✔; T4 links rückwärts, rechts vorwärts ✔; T6 Stopp bei Agent-Abbruch ✔.

**Beobachtung mehrere Sender:** Liefen T2 und T4 gleichzeitig in zwei Terminals, wechselte das linke Rad zwischen vorwärts (+4) und rückwärts (−3); nach Beenden eines Senders folgte der Roboter dem verbleibenden. Erklärung: Ein Topic erlaubt mehrere Publisher ohne Vorrangregel. Regel: nur ein Sender auf `/cmd_vel` (Prüfung: `ros2 topic info /cmd_vel`).

### 08.10.2026 – 2.3 Stufe 9b: Kalibrierung und Fahrfigur auf dem Boden

**Werkzeug:** Node `calib_drive` (Paket `shape_driver`): Startverzögerung 3 s, konstanter Twist für eine feste Dauer, danach 5 Nullbefehle (sauberer Stopp unabhängig vom ESP32-Timeout); bei Strg+C Nullbefehl. Aufbau: glatter Boden, WLAN, Akku; Messung mit Maßband bzw. Sichtprüfung an Klebebandmarkierungen.

**⚠ Zwischenfall:** Erste Fahrt ohne Bewegung – ESP32 nicht verbunden (`ros2 node list` leer, `/cmd_vel` unbekannt). Lösung: Agent starten, EN, `session established` abwarten.

| Fahrt | Befehl | Soll | Ist | Abweichung |
|---|---|---|---|---|
| K1 | `linear_speed 0.031447`, 31,80 s | 1000 mm geradeaus | 995 mm, keine sichtbare Seitenabweichung | −0,5 % |
| K2 | `angular_speed 0.381171`, 16,48 s | 360° | ≈ 355° (geschätzt, ±2°) | ≈ −1,4 % |

**Interpretation:** K1 entspricht einem effektiven Raddurchmesser ≈ 81,6 mm – innerhalb der Messunsicherheit (Lineal ±1 mm ≙ ±1,2 %; Start/Stopp im 50-ms-Raster ≙ ±1,6 mm je Ende). Gleiche wirksame Durchmesser links/rechts. K2 deutet zusätzlich auf eine wirksame Spurweite ≈ 166,5 mm statt 165 mm (Berührpunkt der Räder).

**Kalibrierungsansatz:** Korrektur nicht über Geometrie in der Firmware (wegen grober `param`-Stufen wirkungslos – Räder bleiben bei `param 4`), sondern empirisch über die Ist-Geschwindigkeiten in `robot.yaml`, mit denen `shape_driver` die Phasendauern berechnet. Die Firmware setzt weiterhin `param 4` um.

**Viereck** (`ros2 run shape_driver shape_driver_node --ros-args --params-file …/config/robot.yaml`; 4 Seiten à 0,30 m, 20 Hz, gegen den Uhrzeigersinn):

| Durchgang | `linear_speed` | `angular_speed` | Richtung am Ende | Abstand Endpunkt–Start |
|---|---|---|---|---|
| 1 | 0,03129 m/s (aus K1) | 0,3760 rad/s (aus K2) | 5–8° zu wenig | ≈ 35 mm |
| 2 | 0,03129 m/s | 0,3692 rad/s (Faktor 353,5/360 aus Durchgang 1) | ≈ 2° zu viel | ≈ 2–3 mm |

**Interpretation:**
- Durchgang 1: ≈ 1,5° Fehlwinkel je Ecke; der aufsummierte Richtungsfehler erklärt den Positionsversatz. Mögliche Ursachen: Unsicherheit der K2-Schätzung, Schrittverluste beim Richtungswechsel des linken Rads (Hypothese, nicht geprüft), Zeitraster 50 ms.
- Durchgang 2: Vorzeichenwechsel des Winkelfehlers → optimaler Wert zwischen 0,3692 und 0,3760 rad/s (rechnerisch ≈ 0,371). Feinkorrektur liegt unterhalb der Ablese- und Fahrtstreuung und wurde nicht vorgenommen. Der sehr kleine Positionsfehler kann teilweise auf gegenseitige Kompensation zurückgehen.
- Einschränkung: Einzelfahrten mit geschätzten Winkeln; keine Aussage zur Streuung.

**Vergleich Simulation / Hardware:**

| | turtlesim (50 Hz) | realer Roboter, kalibriert |
|---|---|---|
| Richtungsfehler nach Viereck | 2,1° (zu viel) | ≈ 2° (zu viel) |
| Abstand zum Start | ≈ 0,04 E (≈ 2 % der Seite) | ≈ 2–3 mm (≈ 1 % der Seite) |
| Hauptursache | Zeitraster | Kalibrierung/Mechanik (vor Kalibrierung ≈ 35 mm) |

**Ergebnis ✔ (Pflichtaufgabe 2.3):** Die Node aus 2.2 steuert ohne Codeänderung – nur mit anderer Parameterdatei – den realen Roboter über micro-ROS (WLAN) und fährt das Viereck. Der Positionsfehler wurde durch empirische Kalibrierung von ≈ 35 mm auf ≈ 3 mm reduziert (Einzelfahrten). Es handelt sich um eine zeitgesteuerte Steuerung ohne Positionsrückführung.

### 08.10.2026 – 2.4 Stufe 10: Auswahl Sensorprojekt

**Entscheidung:** Ultraschallsensor HC-SR04 zur Hinderniserkennung. Aufteilung gemäß Aufgabenstellung: Erfassung auf dem ESP32 (Messung, Veröffentlichung als `sensor_msgs/Range`), Auswertung auf dem Host (Filterung, Entscheidung, Eingriff in die Fahrbefehle). Gliederung: 10a Messung, 10b Hinderniserkennung, 10c Reaktionszeit, 10d Buzzer (optional).

### 08.10.2026 – 2.4 Stufe 10a: Ultraschallmessung (HC-SR04)

**Umsetzung (Firmware v3, `firmware/robot_base`):**
- Messung per Interrupt (`CHANGE` auf ECHO, Zeitstempel der steigenden und fallenden Flanke) statt `pulseIn()`, weil `pulseIn()` bis zu 40 ms blockiert und damit `spin_some` und den 50-ms-Regeltakt stören würde. Einzige aktive Wartezeit: 10-µs-Triggerimpuls.
- Zustandsmaschine IDLE/WAIT_ECHO, Soll-Takt 100 ms (`lastTrigger += PERIOD`, wie in Stufe 9 gegen Drift).
- Sonderwerte nach REP 117: `+inf` = Echo länger als Messbereich (kein Objekt), `−inf` = kürzer als 2 cm, `nan` = gar kein Echo (Sensorfehler).
- Publisher `/range` (`sensor_msgs/Range`, `frame_id` `ultrasonic_link`); Zeitstempel über `rmw_uros_sync_session` / `rmw_uros_epoch_nanos()`.

**Funktionstest (WLAN, USB-Versorgung, Roboter steht):**
- `ros2 topic hz /range`: 10,000 Hz, min 0,030 s, max 0,164 s, σ 0,015 s (Fenster 128). Die Streuung entsteht vermutlich bei der WLAN-Übertragung (Takt auf dem ESP32 fest; nicht gemessen).
- `header.stamp` gesetzt (Zeitabgleich funktioniert). Annäherung mit der Hand: stetig 0,36 m → 0,049 m und zurück.
- Ruhewerte springen zwischen festen Stufen (z. B. 0,1470 / 0,1504 / 0,1605 m bei 15 cm), innerhalb einer Stufe ±0,17 mm (= 1 µs).

**Genauigkeit** (Buch senkrecht, Maßband ab Vorderkante der Sensorkapseln, je 30 Werte, Auswertung per Pipeline `ros2 topic echo /range --field range | … | python3` → Median):

| Soll | Median | Abweichung | Spannweite (max − min) |
|---|---|---|---|
| 10 cm | 0,1091 m | +0,9 cm | 0,2 mm |
| 15 cm | 0,1504 m | +0,4 cm | ≈ 14 mm (Stufensprünge, Einzelwerte) |
| 20 cm | 0,2089 m | +0,9 cm | 0,2 mm |
| 30 cm | 0,3008 m | +0,1 cm | 5,5 mm |
| 50 cm | 0,4991 m | −0,1 cm | 0,3 mm |
| 100 cm | 0,9855 m | −1,5 cm | 4,6 mm |
| frei (Tischkante ≈ 1,2 m) | 1,2019 m | – | 4,6 mm |

Ein weiterer Lauf (Median 1,058 m, min 0,573 m) konnte keinem Abstand zugeordnet werden und wurde nicht ausgewertet.

**Ergebnis ✔:** Bis 50 cm ±1 cm, bei 100 cm −1,5 cm; für Hinderniserkennung (Schwellen 20–25 cm) ausreichend.

**Hypothesen (nicht geprüft):**
- Stufensprünge: Empfänger löst bei unterschiedlichen Schwingungen des 40-kHz-Echos aus (Abstände passen nur teilweise zu 25 µs ≙ 4,3 mm) oder ISR-Latenz durch WLAN.
- Abweichungen: Bezugspunkt liegt in der Kapsel statt an der Vorderkante (fester Versatz); Temperaturabhängigkeit der Schallgeschwindigkeit (abstandsproportional).

**Nicht getestet:** `inf`-Fall (kein Freiraum > 4 m); Pegel ECHO (5 V an 3,3-V-Eingang) formal.

### 08.10.2026 – 2.4 Stufe 10b: Hinderniserkennung mit Sicherheitsfilter (Host)

**Problem:** Auf `/cmd_vel` darf nur ein Publisher aktiv sein (Erfahrung Stufe 9). Eine Stopp-Node parallel zur Fahr-Node würde abwechselnd Fahr- und Nullbefehle erzeugen.

**Konzept:** Sicherheitsfilter („Gate“) zwischen Fahr-Node und Roboter:

```
shape_driver / calib_drive ──► /cmd_vel_raw ──► [obstacle_stop] ──► /cmd_vel ──► ESP32
                                                       ▲
                                         /range ───────┘
```

- Fahr-Nodes werden nur per Parameter umgeleitet (`-p cmd_vel_topic:=/cmd_vel_raw`), ohne Codeänderung.
- `obstacle_stop` gibt Befehle unverändert weiter; bei Sperre wird nur `linear.x > 0` auf 0 gesetzt. Drehen und Rückwärtsfahren bleiben erlaubt (Roboter kann sich befreien).
- Medianfilter über 3 Werte gegen Stufensprünge.
- Hysterese: Sperre bei Median < 0,20 m, Freigabe erst bei > 0,25 m (kein Flattern an der Grenze).
- Fail-safe: gesperrt bis zum ersten Messwert, bei > 0,5 s ohne `/range`-Wert, bei `−inf` und `nan` (als Abstand 0 behandelt).
- Dateien: `shape_driver/obstacle_stop.py`, `config/obstacle_stop.yaml`, Entry Point in `setup.py`.

**⚠ Fehler und Lösungen:**
- `RCLError: publisher's context is invalid` bei Strg+C: Humble beendet den ROS-Kontext bei SIGINT selbst; der anschließende Stoppbefehl im `finally` scheitert. Lösung: Stoppbefehl nur bei `rclpy.ok()`; sonst stoppt der Firmware-Timeout (500 ms). Eigener Programmierfehler.
- `TabError` nach manueller Änderung (Tabs und Leerzeichen gemischt) → Datei vollständig ersetzt.
- Korrektur zunächst wirkungslos: Browser speicherte die neue Fassung als `obstacle_stop(1).py`, kopiert wurde die alte. Lösung: Inhalt per `grep` prüfen, alte Downloads löschen.

**Test 1 – Trockentest** (WLAN, USB, Motoren im WLAN-Modus über USB ohne Funktion; Testbefehl `ros2 topic pub -r 10 /cmd_vel_raw …`):

| Prüfung | Ergebnis |
|---|---|
| Hand < 20 cm → `linear.x` 0.0 | ✔ Auslösung bei 0,14–0,20 m |
| Hysterese: Freigabe erst > 0,25 m | ✔ kleinster Freigabewert 0,263 m |
| EN-Reset bzw. Agent gestoppt → Timeout-Sperre | ✔ Warnung, `linear.x` 0.0; nach Wiederverbindung frei |
| Drehbefehl bei Sperre | ✔ `angular.z` 0,3 bleibt, nur `linear.x` → 0.0 |
| `ros2 topic info /cmd_vel` | ✔ Publisher count 1, Subscription count 2 |
| Beenden mit Strg+C | ✔ nach Korrektur ohne Fehler |

**Test 2 – aufgebockt, Akku, USB getrennt:** Räder drehen vorwärts, stoppen sofort bei Hand < 20 cm, laufen bei Freigabe wieder an, stoppen nach Ende der Befehle (Firmware-Timeout) ✔.

**Test 3 – Boden** (`calib_drive` mit `linear_speed 0.031447`, `duration 25` → Soll 0,79 m, über `/cmd_vel_raw`; Start 60 cm vor einer glatten Wand, Sensorwerte mitgeschrieben):
- Startwert 0,6016 m (Markierung 60 cm). Während der Fahrt sinkt der Wert um ≈ 3 mm je Messung (≈ 30 mm/s, passt zu 31,4 mm/s). Einzelne Stufensprünge nach oben wurden vom Medianfilter abgefangen, keine Fehlauslösung.
- Auslösung bei Median 0,200 m (Rohwerte 0,2031 → 0,1999 → 0,1967), danach ein weiterer Messwert 0,1935 m, dann konstant 0,1934–0,1936 m → Stillstand.
- Endabstand Maßband = Sensorwert (≈ 19,4 cm), Roboter gerade zur Wand ✔.
- Nachlauf ≈ 3 mm ab Auslösung bzw. ≈ 6 mm ab erstem Rohwert < 0,20 m → Reaktionszeit grob ≈ 0,2 s (genaue Messung: Stufe 10c).
- Zwei weitere Fahrten unter gleichen Bedingungen bestätigten das Ergebnis (Einzelwerte nicht notiert).

**Störfälle (Vorversuche, Grenzen des Ultraschalls):**

| Versuch | Endabstand (Maßband) | Beobachtete Ursache |
|---|---|---|
| 1 | 13 cm | schlecht reflektierendes Hindernis |
| 3 | 28 cm | kleines Objekt auf der Fahrstrecke im Schallkegel |

Ursachen beobachtet, nicht gezielt nachgemessen. Folgerung: Der Sensor erkennt nur, was ausreichend Schall zurückwirft, und reagiert auf alles im ≈ 15° breiten Kegel – auch auf Objekte neben oder unter der Fahrlinie.

**Ergebnis ✔:** Auswertung auf dem Host stoppt den realen Roboter zuverlässig vor einer Wand (≈ 19 cm bei Schwelle 20 cm) bei unverändertem Fahrprogramm. Die Fahr-Node bleibt unverändert; die Sicherheitslogik ist als eigene Node austauschbar.

---

## Offene Punkte

**Technisch**
- [ ] Stufe 10c: Reaktionszeit Hindernis → Stopp über Zeitstempel messen (Abschätzung aus 10b: ≈ 0,2 s)
- [ ] Stufe 10d (optional): Buzzer als akustische Warnung
- [ ] Ultraschall: Ursache der Stufensprünge (3–12 mm) klären – z. B. Vergleich serieller Transport ohne WLAN
- [ ] Ultraschall: Pegel ECHO (5 V an 3,3-V-Eingang) prüfen (Datenblatt/Messung, ggf. Spannungsteiler)
- [ ] Ultraschall: inf-Fall praktisch testen (Freiraum > 4 m) – optional
- [ ] Optional: Wiederholungen von K1/K2/Viereck zur Bestimmung der Streuung; K2 über 2 Umdrehungen
- [ ] Optional: Hypothese Schrittverluste beim Richtungswechsel prüfen (Pause zwischen Phasen)
- [ ] Automatische Wiederverbindung zum Agent (derzeit Reset per EN nötig) – optional
- [ ] `param 1` (schnellste Stufe) und Verhalten unter Last prüfen (Schrittverluste?)
- [ ] Verhalten mit Akku an und USB gleichzeitig klären (Versorgung, Wirkung des Akkuschalters)
- [ ] Anfrage an Betreuung zum ATtiny-Protokoll versenden / Antwort mit Versuchsergebnissen abgleichen
- [ ] Optional: Gegenprobe Int32-Publisher mit `delay(10)` (durch Stufe 6b bereits stark gestützt)
- [ ] Ursache der verfälschten Boot-ROM-Ausgabe (derzeit ohne Auswirkung)

**Dokumentation / Repository**
- [ ] Stoppverhalten nach Abbruch von `--rate 1` (turtlesim) bestätigen
- [ ] rqt_graph-Screenshot mit vollständiger Kette – optional
- [ ] Gegenprobe Parameterdatei: `num_sides: 3` → Dreieck – optional
- [ ] Eingecheckte generierte Dateien auf `main` (`a.out`, Binärdateien, `__pycache__`) mit Projektpartnerin klären
- [ ] Zeitpunkt der Zusammenführung von `leo` nach `main` abstimmen
- [ ] Mailadresse im öffentlichen Repo bestätigen (aktuell Hochschuladresse in `package.xml`/`setup.py`)

## Quellen

- Praktikumsanleitung „Praktikum_2026“, Abschnitte 2.1–2.3, 4.1.1–4.1.4
- ROS 2 Humble – Installation Ubuntu (deb packages): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- turtlesim Quellcode (Humble): https://github.com/ros/ros_tutorials/blob/humble/turtlesim/src/turtle.cpp
- ROS 2 Humble – Creating a package: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- ROS 2 Humble – Integrating launch files into ROS 2 packages: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html
- micro_ros_setup (Branch humble): https://github.com/micro-ROS/micro_ros_setup/tree/humble
- micro_ros_platformio: https://github.com/micro-ROS/micro_ros_platformio
- PlatformIO – udev rules: https://docs.platformio.org/en/latest/core/installation/udev-rules.html
- PlatformIO – Install Shell Commands: https://docs.platformio.org/en/latest/core/installation/shell-commands.html
- REP 117 – Informational Distance Measurements (Sonderwerte inf/−inf/nan): https://www.ros.org/reps/rep-0117.html

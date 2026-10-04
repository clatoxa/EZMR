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
| 2.3 | ESP32-Roboter über micro-ROS | ⏳ in Arbeit: ESP32 flashbar, micro-ROS-Verbindung ✔; Bewegungsbefehle/Motoren offen |
| 2.4 | Sensorprojekt (Erfassung ESP32, Auswertung Host) | ⏳ |
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
| 6 | Twist-Subscriber auf dem ESP32, Empfang ohne Motoren | 2.3 | ⏳ als Nächstes |
| 7 | Motorprotokoll: Einzelbefehle über UART an die ATtiny212 | 2.3 | ⏳ |
| 8 | Differentialantrieb: Twist → Radgeschwindigkeiten | 2.3 | ⏳ |
| 9 | `shape_driver` auf dem realen Roboter | 2.3 | ⏳ |
| 10 | Sensorintegration | 2.4 | ⏳ |
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

---

## Offene Punkte

**Technisch**
- [ ] Stufe 6: Twist-Subscriber auf dem ESP32 (Empfang ohne Motoren nachweisen)
- [ ] Gegenprobe Int32-Publisher mit `delay(10)`, längere Messung (≥ 60 Intervalle)
- [ ] Versionen in `platformio.ini` festschreiben und mit `pio pkg list` prüfen (`sha.cfee17f`)
- [ ] Motorprotokoll klären (Voraussetzung für Stufe 7): UART-Baudrate, Motor-IDs, Codierung der Drehrichtung, Byte-Reihenfolge des 16-Bit-Parameters, Geschwindigkeitseinheit, gültiger Stoppbefehl → Quellcode der ATtiny-Controller oder funktionierendes Beispiel bei Betreuung erfragen
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

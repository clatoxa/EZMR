# EZMR – Echtzeitsysteme und mobile Robotik

Quellcode und Dokumentation zum Praktikum „Echtzeitsysteme und mobile Robotik“ (HTWK Leipzig).

## Inhalt

| Verzeichnis | Inhalt |
|---|---|
| `ros2_ws/src/shape_driver` | ROS-2-Paket (Python): zeitgesteuerte Fahrfolge (Vieleck) über `geometry_msgs/Twist`, in turtlesim getestet |
| `docs/` | Arbeitsprotokoll mit Versuchen und Ergebnissen |
| `Programmieraufgaben/` | Programmieraufgaben (C) |
| `Turtlebot/` | Arbeiten mit dem TurtleBot |
| `firmware/` | PlatformIO-Projekt für den ESP32 (folgt) |

## Voraussetzungen

- Ubuntu 22.04
- ROS 2 Humble (`ros-humble-desktop`, `ros-dev-tools`)

## Bauen und Starten

```bash
cd ros2_ws
colcon build --symlink-install
source install/local_setup.bash
ros2 launch shape_driver turtlesim_shape_launch.py
```

Die Fahrparameter stehen in `ros2_ws/src/shape_driver/config/turtlesim.yaml`.
Eine andere Parameterdatei lässt sich mit `params_file:=<pfad>` übergeben.

## Lizenz

Apache-2.0 (siehe `LICENSE`).

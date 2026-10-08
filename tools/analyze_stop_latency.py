#!/usr/bin/env python3
"""Stufe 10c: Reaktionszeit Hindernis -> Stopp aus einer rosbag2-Aufzeichnung bestimmen.

Aufzeichnung (Humble, Standard-Speicherformat sqlite3):
    ros2 bag record -o <ordner> /range /cmd_vel_raw /cmd_vel /wheel_cmd

Auswertung (ROS-Umgebung gesourct):
    python3 analyze_stop_latency.py <ordner> [--stop 0.20] [--release 0.25] [--window 3]

Zeitbasis: Empfangszeit beim Rekorder (Host-Uhr). Zusätzlich header.stamp von /range
(ESP32-Uhr, per rmw_uros_sync_session an den Agent-Host angeglichen).

Ereigniskette je Stopp:
  E0  erster Rohwert < stop       (header.stamp = Messzeitpunkt auf dem ESP32)
  E1  derselbe Wert kommt am Host an
  E2  Median < stop -> Sperre      (Nachbildung von obstacle_stop)
  E3  erstes /cmd_vel mit linear.x == 0, während /cmd_vel_raw vorwärts fordert
  E4  erstes /wheel_cmd mit x == 0 und y == 0 (ESP32 hat Motoren gestoppt und meldet das)

Stand 08.10.2026: UNGETESTET (Syntax geprüft, keine ROS-Umgebung beim Erstellen).
"""
import argparse
import math
import statistics
from collections import deque

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ('/range', '/cmd_vel_raw', '/cmd_vel', '/wheel_cmd')


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in TOPICS if t not in types]
    if missing:
        raise SystemExit(f'Fehlende Topics in der Aufzeichnung: {missing}')
    classes = {name: get_message(types[name]) for name in TOPICS}
    events = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in classes:
            events.append((t_ns * 1e-9, topic, deserialize_message(data, classes[topic])))
    events.sort(key=lambda e: e[0])
    return events


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def ms(a, b):
    return f'{(b - a) * 1000:7.1f} ms' if a is not None and b is not None else '   –'


def analyze(events, stop, release, window):
    buf = deque(maxlen=window)
    recent = deque(maxlen=6)        # letzte Rohwerte (zur Plausibilitätsprüfung)
    t0 = events[0][0] if events else 0.0
    blocked = True                  # wie obstacle_stop: bis zur ersten Freigabe gesperrt
    raw_forward = False             # fordert die Fahr-Node gerade Vorwärtsfahrt?
    first_below = None              # (t_host, stamp, wert) des ersten Rohwerts < stop seit Freigabe
    results = []
    current = None                  # laufender Stoppvorgang

    for t, topic, msg in events:
        if topic == '/cmd_vel_raw':
            raw_forward = msg.linear.x > 0.0

        elif topic == '/range':
            r = msg.range
            if math.isnan(r) or r < 0.0:
                r = 0.0
            # E0 = Beginn der letzten ununterbrochenen Folge von Rohwerten < stop
            # (einzelne Ausreißer nach unten, die wieder über stop zurückgehen, zählen nicht)
            if not blocked:
                if r >= stop:
                    first_below = None
                elif first_below is None:
                    first_below = (t, stamp_s(msg), r)
            buf.append(r)
            recent.append(r)
            med = statistics.median(buf)
            if blocked and med > release:
                blocked = False
                first_below = None
            elif not blocked and med < stop:
                blocked = True
                if raw_forward:     # nur Stopps während einer Fahrt auswerten
                    current = {'e0_stamp': first_below[1] if first_below else None,
                               'e1': first_below[0] if first_below else None,
                               'e0_value': first_below[2] if first_below else None,
                               'e2': t, 'e2_stamp': stamp_s(msg), 'e2_median': med,
                               't_rel': t - t0, 'raw_before': list(recent),
                               'e3': None, 'e4': None}
                    results.append(current)
                first_below = None

        elif topic == '/cmd_vel' and current is not None and current['e3'] is None:
            if raw_forward and msg.linear.x == 0.0:
                current['e3'] = t

        elif topic == '/wheel_cmd' and current is not None and current['e3'] is not None \
                and current['e4'] is None:
            if msg.x == 0.0 and msg.y == 0.0:
                current['e4'] = t
    return results


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('bag')
    p.add_argument('--stop', type=float, default=0.20)
    p.add_argument('--release', type=float, default=0.25)
    p.add_argument('--window', type=int, default=3)
    a = p.parse_args()

    events = read_bag(a.bag)
    counts = {t: sum(1 for e in events if e[1] == t) for t in TOPICS}
    print('Nachrichten:', ', '.join(f'{k} {v}' for k, v in counts.items()))

    results = analyze(events, a.stop, a.release, a.window)
    if not results:
        print('Kein Stopp während einer Vorwärtsfahrt gefunden.')
        return
    for i, r in enumerate(results, 1):
        print(f'\nStopp {i} (t = {r["t_rel"]:.1f} s nach Aufnahmebeginn): erster Rohwert < {a.stop} m = '
              f'{r["e0_value"]}, Median bei Sperre = {r["e2_median"]:.4f} m')
        print('  letzte Rohwerte bis zur Sperre: ' + ', '.join(f'{v:.3f}' for v in r['raw_before']))
        print(f'  E0->E1  Messung ESP32 -> Ankunft Host (Uhrenabgleich!) {ms(r["e0_stamp"], r["e1"])}')
        print(f'  E1->E2  Medianfilter (Wartezeit auf weitere Werte)     {ms(r["e1"], r["e2"])}')
        print(f'  E2->E3  Sperre -> Nullbefehl auf /cmd_vel              {ms(r["e2"], r["e3"])}')
        print(f'  E3->E4  /cmd_vel -> /wheel_cmd 0 zurück am Host        {ms(r["e3"], r["e4"])}')
        print(f'  E0->E4  gesamt (Messung bis Stopp gemeldet)            {ms(r["e0_stamp"], r["e4"])}')
        print(f'  E1->E4  gesamt nur Host-Uhr (ohne Uhrenabgleich)       {ms(r["e1"], r["e4"])}')


if __name__ == '__main__':
    main()

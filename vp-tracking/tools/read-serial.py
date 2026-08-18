#!/usr/bin/env python3
"""Lecture bornee du flux serie — se termine toujours, contrairement au REPL.

    ./read-serial.py --lines 50
    ./read-serial.py --seconds 10 --filter E:
"""
import argparse, sys, time

p = argparse.ArgumentParser()
p.add_argument("--port", default="/dev/vp_encoders")
p.add_argument("--baud", type=int, default=115200)
p.add_argument("--lines", type=int, default=0, help="0 = illimite")
p.add_argument("--seconds", type=float, default=10.0)
p.add_argument("--filter", default=None, help="ne garder que les lignes commencant par ceci")
a = p.parse_args()

import serial
try:
    ser = serial.Serial(a.port, a.baud, timeout=0.5)
except Exception as e:
    print("Erreur: %s" % e, file=sys.stderr); sys.exit(1)

deadline = time.monotonic() + a.seconds
count = 0
while time.monotonic() < deadline:
    line = ser.readline().decode("utf8", "replace").rstrip()
    if not line:
        continue
    if a.filter and not line.startswith(a.filter):
        continue
    print(line)
    count += 1
    if a.lines and count >= a.lines:
        break
ser.close()
print("# %d lignes" % count, file=sys.stderr)

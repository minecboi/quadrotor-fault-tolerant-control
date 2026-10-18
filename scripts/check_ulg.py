#!/usr/bin/env python3
"""check_ulg.py - confirm inject.py's markers are inside a PX4 .ulg and compare with the JSON.
Usage: python3 check_ulg.py LOG.ulg [inject_log.json]        (needs: pip install pyulog)
Marker fields: x = motor, y = fault code (0 complete,1 toggle,2 power,3 prop), z = level.
"""
import sys, json
from pyulog import ULog

u = ULog(sys.argv[1])
names = sorted({d.name for d in u.data_list})
if "debug_vect" not in names:
    sys.exit("NO debug_vect topic in this log. Topics present: " + ", ".join(names) +
             "\n-> check logger_topics.txt location and restart SITL.")
d = next(x for x in u.data_list if x.name == "debug_vect")
t0 = u.start_timestamp
print(f"log start timestamp: {t0} us ; {len(d.data['timestamp'])} marker(s)")
for ts, x, y, z in zip(d.data["timestamp"], d.data["x"], d.data["y"], d.data["z"]):
    print(f"  ulog t={ts} us ({(ts - t0) / 1e6:7.3f} s into log)  motor={x:.0f} fault={y:.0f} level={z:.2f}")
if len(sys.argv) > 2:
    j = json.load(open(sys.argv[2]))
    print("JSON t_inject_us =", j["t_inject_us"], "(sim boot time; compare with first marker above)")
    

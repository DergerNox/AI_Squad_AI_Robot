"""approach from different starting places on the twin, with colour-based vision.

For each start: the steps it took, the time, and where the car really ended up
(from Gazebo): distance from the bumper to the target's surface, and its direction.
"""
import math
import os
import re
import subprocess
import sys
import time

D = os.path.abspath(sys.argv[1])
sys.path.insert(0, D)
sys.path.insert(1, os.path.dirname(os.path.abspath(__file__)))
os.chdir(D)
import rclpy  # noqa: E402
from rai.communication.ros2 import ROS2Connector  # noqa: E402

import picar_tools as pt  # noqa: E402
from color_vision import ColourVision  # noqa: E402

TARGETS = {"red box": ((1.0, 0.0), 0.1), "blue cylinder": ((-0.6, 0.8), 0.1),
           "yellow box": ((0.4, -0.9), 0.1)}
SONAR_X = 0.145 / 2 + 0.220 / 2 + 0.006


def gz(*args):
    return subprocess.run(["gz", *args], capture_output=True, text=True, timeout=15).stdout


def place(x, y, yaw_deg):
    yaw = math.radians(yaw_deg)
    gz("service", "-s", "/world/picar_room/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype",
       "gz.msgs.Boolean", "--timeout", "3000",
       "--req", f'name: "picar", position: {{x: {x}, y: {y}, z: 0.0}}, '
                f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
    time.sleep(2.0)


def truth(target):
    out = gz("model", "-m", "picar", "-p")
    x, y, _, _, _, yaw = [float(v) for v in re.findall(r"-?\d+\.\d+", out.split("Pose")[-1])[:6]]
    (tx, ty), radius = TARGETS[target]
    sx, sy = x + SONAR_X * math.cos(yaw), y + SONAR_X * math.sin(yaw)
    gap = math.hypot(tx - sx, ty - sy) - radius
    bearing = math.degrees(math.atan2(ty - sy, tx - sx) - yaw)
    bearing = (bearing + 180) % 360 - 180
    return gap, bearing


rclpy.init()
conn = ROS2Connector()
vision = ColourVision()
tools, new_task, memory = pt.make_picar_tools(conn, vision)
T = {t.name: t for t in tools}

STARTS = [
    ("far, straight ahead", (-0.6, 0.0, 0), "red box"),
    ("off to one side", (-0.2, 0.45, 0), "red box"),
    ("close and to the side", (0.45, 0.22, 0), "red box"),
    ("very close, edge of the picture", (0.38, 0.30, 0), "red box"),
    ("blue cylinder, behind-left", (0.3, 0.0, 150), "blue cylinder"),
    ("yellow box, angled", (-0.4, -0.5, -20), "yellow box"),
]
for name, (x, y, yaw), target in STARTS:
    place(x, y, yaw)
    new_task()
    gap0, bearing0 = truth(target)
    start, calls = time.time(), vision.calls
    out = T["approach"].invoke({"name": "approach", "args": {"target": target}, "id": "c", "type": "tool_call"})
    gap, bearing = truth(target)
    print(f"\n== {name}: {target} starts {gap0:.2f} m away at {bearing0:+.0f} deg")
    print(f"   {time.time() - start:.1f} s, {vision.calls - calls} vision questions")
    print(f"   -> {out.content}")
    print(f"   ended {gap:.2f} m from it at {bearing:+.0f} deg")

conn.shutdown()
rclpy.shutdown()

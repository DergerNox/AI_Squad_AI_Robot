"""safety.py: the SafetyGate, the rules every drive command passes through.

speed is motor power from -1.0 to 1.0; positive is forward. The gate stops the car when
the emergency stop is on or the commands stop coming, and stops forward driving when a
distance sensor sees something too close or has sent no reading for a while. Reversing
is always allowed.
"""

import math
import time


class SafetyGate:

    def __init__(self, max_speed, cmd_timeout_s, obstacle_stop_m, range_timeout_s,
                 sensors=("range",), clock=None):
        """sensors: the names of the distance sensors that must all see a free way ahead.
        clock: returns the time in seconds (default time.monotonic)."""
        self.max_speed = max_speed
        self.cmd_timeout_s = cmd_timeout_s
        self.obstacle_stop_m = obstacle_stop_m
        self.range_timeout_s = range_timeout_s
        self.clock = clock or time.monotonic
        self.last_cmd_t = -math.inf    # never: no command has arrived yet
        # sensor name -> (metres, when): nothing seen, never read
        self.ranges = {name: (math.inf, -math.inf) for name in sensors}
        self.estop = False

    def on_command(self):
        self.last_cmd_t = self.clock()

    def on_range(self, meters, sensor="range"):
        """A reading in metres. REP 117: +inf = nothing in range, -inf = too close."""
        self.ranges[sensor] = (meters, self.clock())

    @property
    def range_m(self):
        """The closest reading of all the sensors."""
        return min(meters for meters, _ in self.ranges.values())

    def filter(self, speed):
        """Return (allowed_speed, reason)."""
        now = self.clock()
        if self.estop:
            return 0.0, "estop"
        if now - self.last_cmd_t > self.cmd_timeout_s:
            return 0.0, "timeout"
        speed = max(-self.max_speed, min(self.max_speed, speed))
        if speed > 0:
            if any(now - when > self.range_timeout_s for _, when in self.ranges.values()):
                return 0.0, "no_range"
            if self.range_m < self.obstacle_stop_m:
                return 0.0, "obstacle"
        return speed, "ok"

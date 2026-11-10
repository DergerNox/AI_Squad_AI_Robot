# picar_base/safety.py
import time


class SafetyGate:
    """speed is motor power from -1.0 to 1.0; positive is forward."""

    def __init__(self, max_speed, cmd_timeout_s, obstacle_stop_m, range_timeout_s):
        self.max_speed = max_speed
        self.cmd_timeout_s = cmd_timeout_s
        self.obstacle_stop_m = obstacle_stop_m
        self.range_timeout_s = range_timeout_s
        self.last_cmd_t = 0.0
        self.last_range_t = 0.0
        self.range_m = float("inf")
        self.estop = False

    def on_command(self):
        self.last_cmd_t = time.monotonic()

    def on_range(self, meters):
        self.range_m = meters  # REP 117: +inf = nothing in range, -inf = too close
        self.last_range_t = time.monotonic()

    def filter(self, speed):
        """Return (allowed_speed, reason)."""
        now = time.monotonic()
        if self.estop:
            return 0.0, "estop"
        if now - self.last_cmd_t > self.cmd_timeout_s:
            return 0.0, "timeout"
        speed = max(-self.max_speed, min(self.max_speed, speed))
        if speed > 0:
            if now - self.last_range_t > self.range_timeout_s:
                return 0.0, "no_range"
            if self.range_m < self.obstacle_stop_m:
                return 0.0, "obstacle"
        return speed, "ok"

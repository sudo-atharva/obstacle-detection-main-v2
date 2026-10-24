"""Speed decision logic. Contains no GUI or camera code, so it can be unit-tested.

The vehicle only drives straight ahead. Each update gets the nearest obstacle the camera sees
(estimated metres, or None) and the nearest ultrasonic reading (cm, or None):

    STOPPED ──Start──▶ CRUISING ──camera obstacle within slow_distance──▶ SLOWING (slow_speed)
    CRUISING / SLOWING ──camera within stop_distance or ultrasonic within sonar_stop_cm──▶ BLOCKED
    BLOCKED / SLOWING ──cause gone for clear_time──▶ faster state again (automatic)
    any state ──Stop button or fault (camera lost, ESP32 / sensor gone)──▶ STOPPED (needs Start)
BLOCKED means speed 0, so the vehicle will not start with something in front of it.
"""
import math
import time
from dataclasses import dataclass
from enum import Enum


class State(Enum):
    STOPPED = "Stopped"
    CRUISING = "Cruising"
    SLOWING = "Obstacle ahead - slowing"
    BLOCKED = "Blocked by obstacle"


@dataclass
class Config:
    cruise_speed: float = 1.0      # speeds are fractions of full speed, 0..1
    slow_speed: float = 0.3        # speed limit while the camera sees an obstacle in range
    slow_distance: float = 8.0     # m: camera obstacle closer than this -> slow_speed
    stop_distance: float = 2.0     # m: camera obstacle closer than this -> stop
    sonar_stop_cm: float = 40.0    # cm: either ultrasonic sensor closer than this -> stop
    clear_time: float = 1.0        # s the cause must be gone before speeding up again (> 0)
    decel_rate: float = 0.8        # speed units per second when slowing
    accel_rate: float = 0.3        # speed units per second when speeding up


class ObstacleController:
    def __init__(self, config: Config | None = None, clock=time.monotonic):
        self.config = config or Config()
        self.clock = clock
        self.running = False
        self.state = State.STOPPED
        self.speed = 0.0
        self.reason = "Press Start"
        self._blocked_at = self._seen_at = -math.inf
        self._last_update = clock()

    def start(self):
        self.running = True

    def stop(self, reason="Stopped by pilot"):
        self.running, self.reason = False, reason

    def update(self, camera_m=None, sonar_cm=None, fault="") -> float:
        """Feed the latest readings; returns the speed to send to the motors."""
        c = self.config
        now = self.clock()
        dt = max(0.0, now - self._last_update)
        self._last_update = now

        if fault and self.running:
            self.stop(fault)
        block = ""
        if camera_m is not None and camera_m <= c.stop_distance:
            block = f"Camera: obstacle at {camera_m:.1f} m"
        elif sonar_cm is not None and sonar_cm <= c.sonar_stop_cm:
            block = f"Ultrasonic: obstacle at {sonar_cm:.0f} cm"
        if block:
            self._blocked_at = now
            if self.running:
                self.reason = block
        if camera_m is not None and camera_m <= c.slow_distance:
            self._seen_at = now

        if not self.running:
            self.state = State.STOPPED
        elif now - self._blocked_at < c.clear_time:
            self.state = State.BLOCKED
        elif now - self._seen_at < c.clear_time:
            self.state = State.SLOWING
        else:
            self.state = State.CRUISING

        target = {State.CRUISING: c.cruise_speed, State.SLOWING: c.slow_speed}.get(self.state, 0.0)
        if target <= 0:
            self.speed = 0.0  # stopping is never gradual
        elif self.speed > target:
            self.speed = max(target, self.speed - c.decel_rate * dt)
        else:
            self.speed = min(target, self.speed + c.accel_rate * dt)
        return self.speed

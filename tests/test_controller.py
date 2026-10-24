import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from controller import Config, ObstacleController, State
from detector import ObstacleDetector
from esp32_link import parse_sonar


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make(**cfg):
    clock = FakeClock()
    ctl = ObstacleController(Config(**cfg), clock=clock)
    return ctl, clock


def run(ctl, clock, seconds, camera_m=None, sonar_cm=None, fault="", step=0.1):
    for _ in range(round(seconds / step)):
        clock.t += step
        ctl.update(camera_m, sonar_cm, fault)


def started():
    ctl, clock = make()
    ctl.start()
    run(ctl, clock, 10, sonar_cm=999)
    return ctl, clock


def test_does_not_move_until_started():
    ctl, clock = make()
    run(ctl, clock, 5)
    assert ctl.state == State.STOPPED and ctl.speed == 0.0


def test_accelerates_to_cruise_when_clear():
    ctl, _ = started()
    assert ctl.state == State.CRUISING and ctl.speed == 1.0


def test_far_obstacle_slows_to_limit():
    ctl, clock = started()
    run(ctl, clock, 3, camera_m=6)
    assert ctl.state == State.SLOWING and ctl.speed == 0.3


def test_obstacle_beyond_slow_distance_is_ignored():
    ctl, clock = started()
    run(ctl, clock, 3, camera_m=20)
    assert ctl.state == State.CRUISING


def test_close_camera_obstacle_stops_then_resumes_when_gone():
    ctl, clock = started()
    run(ctl, clock, 0.1, camera_m=1.5)
    assert ctl.state == State.BLOCKED and ctl.speed == 0.0
    run(ctl, clock, 0.5)
    assert ctl.state == State.BLOCKED  # must stay clear for clear_time
    run(ctl, clock, 1)
    assert ctl.state == State.CRUISING


def test_ultrasonic_blocks_start():
    ctl, clock = make(sonar_stop_cm=40)
    ctl.start()
    run(ctl, clock, 5, sonar_cm=30)
    assert ctl.state == State.BLOCKED and ctl.speed == 0.0
    assert "Ultrasonic" in ctl.reason


def test_fault_stops_and_needs_start_again():
    ctl, clock = started()
    run(ctl, clock, 0.1, fault="No camera signal")
    assert ctl.state == State.STOPPED and ctl.reason == "No camera signal"
    run(ctl, clock, 5)
    assert ctl.state == State.STOPPED
    ctl.start()
    run(ctl, clock, 0.1)
    assert ctl.state == State.CRUISING


def test_pilot_stop():
    ctl, clock = started()
    ctl.stop()
    run(ctl, clock, 0.1)
    assert ctl.state == State.STOPPED and ctl.speed == 0.0


def test_parse_sonar():
    assert parse_sonar(b"U123,85\r") == (123.0, 85.0)
    assert parse_sonar(b"speed 0.30") is None
    assert parse_sonar(b"U12") is None


def test_bigger_box_means_closer():
    det = ObstacleDetector(backend="edges")
    frame = np.zeros((480, 640, 3), np.uint8)
    det._edges = lambda f: [(300, 100, 340, 340, "object", 1.0)]  # 240 px tall
    near = det.detect(frame)[0][0][0]
    det._edges = lambda f: [(300, 200, 340, 260, "object", 1.0)]  # 60 px tall
    far = det.detect(frame)[0][0][0]
    assert near < far
    assert abs(near - 1.0 * 1.2 * 480 / 240) < 1e-9


def test_non_obstacles_and_off_path_are_not_counted():
    det = ObstacleDetector(backend="edges")
    frame = np.zeros((480, 640, 3), np.uint8)
    det._edges = lambda f: [(300, 100, 340, 340, "chair", 0.9), (0, 100, 40, 340, "person", 0.9)]
    obstacles, others = det.detect(frame)
    assert obstacles == [] and [o[0] for o in others] == ["chair", "person"]

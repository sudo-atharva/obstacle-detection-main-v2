"""Finds the nearest obstacle in the vehicle's path and estimates its distance.

Only the "path zone", a band in the middle of the image, counts. Distance comes from the box
height: an object of real height H metres that is h pixels tall is about H * f / h metres away,
f being the focal length in pixels. Big box = close, small box = far.

Calibrating `focal` (fraction of frame height): stand a person at a known distance d metres,
note the box height b pixels in a frame of height F pixels, then focal = d * b / (1.7 * F).
An object cut off by the frame edge looks smaller, so farther, than it is: the ultrasonic
sensors cover that close range.

Backends:
  YOLO  : (pip install ultralytics) names every object it sees (80 COCO classes). Only people,
          vehicles and animals in the path count as obstacles; the rest is only shown.
  EDGES : needs no model. Any large outlined shape in the path zone counts as a 1 m tall object.
          Rough, but fine for testing.
"""
import cv2
import numpy as np

# Typical real heights (m) of the obstacles we care about.
REAL_HEIGHT_M = {
    "person": 1.7, "car": 1.5, "truck": 3.0, "bus": 3.0, "motorcycle": 1.2, "bicycle": 1.1,
    "cow": 1.4, "horse": 1.6, "sheep": 0.9, "dog": 0.6, "cat": 0.3, "bird": 0.3,
    "elephant": 3.0, "bear": 1.2, "zebra": 1.4, "giraffe": 4.5,
    "object": 1.0,  # edges backend
}


class ObstacleDetector:
    def __init__(self, backend="auto", path_width=0.4, focal=1.2):
        self.path_width = path_width  # fraction of frame width treated as "our path"
        self.focal = focal            # focal length / frame height; see calibration above
        self.model = None
        if backend in ("auto", "yolo"):
            try:
                from ultralytics import YOLO
                self.model = YOLO("yolov8n.pt")
            except ImportError:
                if backend == "yolo":
                    raise
        self.name = "YOLO" if self.model else "Edges"

    def path_zone(self, frame):
        w = frame.shape[1]
        half = int(w * self.path_width / 2)
        return w // 2 - half, w // 2 + half

    def detect(self, frame):
        """Returns (obstacles, others).

        obstacles: (distance m, label, confidence, box) for obstacles in the path, nearest first.
        others:    (label, confidence, box) for everything else seen (other classes, outside the path).
        """
        h = frame.shape[0]
        zx1, zx2 = self.path_zone(frame)
        obstacles, others = [], []
        for x1, y1, x2, y2, label, conf in self._yolo(frame) if self.model else self._edges(frame):
            box = (x1, y1, x2, y2)
            if label in REAL_HEIGHT_M and x2 >= zx1 and x1 <= zx2 and y2 > y1:
                obstacles.append((REAL_HEIGHT_M[label] * self.focal * h / (y2 - y1), label, conf, box))
            else:
                others.append((label, conf, box))
        return sorted(obstacles), others

    def _yolo(self, frame):
        result = self.model(frame, conf=0.4, verbose=False)[0]
        names = result.names
        return [
            (*map(int, xyxy), names[int(cls)], conf)
            for xyxy, cls, conf in zip(result.boxes.xyxy.tolist(), result.boxes.cls.tolist(),
                                       result.boxes.conf.tolist())
        ]

    def _edges(self, frame):
        gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        edges = cv2.Canny(gray, 50, 150)
        edges = cv2.dilate(edges, np.ones((9, 9), np.uint8), iterations=2)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = frame.shape[0] * frame.shape[1] * 0.03
        out = []
        for c in contours:
            if cv2.contourArea(c) < min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            out.append((x, y, x + w, y + h, "object", 1.0))
        return out


def draw_overlay(frame, detector, obstacles, others, config):
    """Path zone, obstacles (red = stop range, orange = slow range, green = farther) and,
    in thin grey, everything else the detector recognised."""
    zx1, zx2 = detector.path_zone(frame)
    overlay = frame.copy()
    cv2.rectangle(overlay, (zx1, 0), (zx2, frame.shape[0]), (255, 200, 0), -1)
    cv2.addWeighted(overlay, 0.12, frame, 0.88, 0, frame)
    for label, conf, (x1, y1, x2, y2) in others:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (170, 170, 170), 1)
        cv2.putText(frame, f"{label} {conf:.0%}", (x1, max(14, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 170, 170), 1)
    for distance, label, conf, (x1, y1, x2, y2) in obstacles:
        color = ((0, 0, 255) if distance <= config.stop_distance
                 else (0, 140, 255) if distance <= config.slow_distance else (0, 200, 0))
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        cv2.putText(frame, f"{label} {conf:.0%}  {distance:.1f} m", (x1, max(18, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return frame

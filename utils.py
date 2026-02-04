# utils.py
from __future__ import annotations
import math

# cars
carInfo = []
zmqPublisherIP = "localhost"
zmqPublisherPort = "5556"  

simSpeed = 3.0
wheelBase = 20.0
tagRatio = 0.878  # (3.6cm) / (4.1cm)  tag size in cm / actual tag size in cm

container_width = 700
container_height = 500
painter_width = 1280
painter_height = 720
spacer = 8
gridCopy = []

# colors
RGB_WHITE = (255, 255, 255)
RGB_BLACK = (0, 0, 0)
RGB_RED = (255, 0, 0)
RGB_GREEN = (0, 255, 0)
RGB_BLUE = (0, 0, 255)
RGB_GREY = (128, 128, 128)

RGB_PATH_COLORS = [
    (0, 0, 0), (1, 0, 103), (213, 255, 0), (255, 0, 86),
    (158, 0, 142), (14, 76, 161), (255, 229, 2), (0, 95, 57),
    (0, 255, 0), (149, 0, 58), (255, 147, 126), (164, 36, 0),
]

class UnitCar:
    def __init__(self, tag="0", ID="0"):
        self.tag = tag
        self.ID = ID
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.lSpeed = 0.0
        self.rSpeed = 0.0
        self.path = []

class Boundary:
    def __init__(self):
        self.u = 2 * wheelBase
        self.d = painter_height - 2 * wheelBase
        self.l = 2 * wheelBase
        self.r = painter_width - 2 * wheelBase
        self.width = self.r - self.l
        self.height = self.d - self.u

def CheckCollosion(thresh, x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2) <= thresh

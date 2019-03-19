"""

Equation to find distance:

tan(a1 + a2) = (h2 - h1) / d

d = (h2 - h1)/tan(a1 + a2)

h2 = height of target (known property)

h1 = height of camera off the floor.

a1 = mounting angle

a2 = angle to target (needs to be computed)

sqrt((x2 - x1)^2 + (y2 - y1)^2)

x2 = contour center
y2 = contour center

x1 = actual center
y1 = actual center

"""
import math
from legacy import contour_tracker


class ComputeDistance:

    def __init__(self):
        self.h2 = 20.875  # height of hub hatch in
        self.h1 = 5 # height of camera in
        self.a1 = 45 # angle of camera in degrees
        self.resolution = (1920, 1080)
        self.contour_center = contour_tracker.Target.average_center
        self.center_screen = self.resolution[0]/2, self.resolution[1]/2
        self.a2 = 0


    def find_distance(self):

        pixel_angle = 45 / (math.sqrt(math.pow(self.resolution[0], 2) + math.pow(self.resolution[1], 2)))
        distance_to_center = math.sqrt(math.pow(self.contour_center[0] - self.center_screen[0], 2) + math.pow(
        self.contour_center[1] - self.center_screen[1], 2))
        self.a2 = distance_to_center * pixel_angle


        distance = (self.h2 - self.h1) / math.tan(self.a1 + self.a2)
        return distance


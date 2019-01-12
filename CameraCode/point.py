from math import *

class Point(object):
    """Creates a new Point (x, y)"""

    def __init__(self, x, y):
        """Defines x and y coordinates"""
        self.x = x
        self.y = y

    def __str__(self):
        return "Point(%s, %s)" % (self.x, self.y)

    def distance(self, other):
        return hypot(self.x - other.get_x, self.y - other.get_y)
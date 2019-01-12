from math import *
from point import Point

class RobotPosition(object):
    def __init__(self, point, yaw):
        self.point = point
        self.yaw = yaw

    def __str__(self):
        return "RobotPosition (%s , %s degrees)" % self.point.__str__(), self.yaw

    @staticmethod
    def translate_rp(rp, d_point):
        """
        Translates a RobotPosition by a Point as a vector, wrt the current yaw of the Robot. I.E. if your heading is 30
        degrees, a shift of (0, 1) will shift rp by (0, 1) in the coordinate plane that is rotated 30 degrees off the
        standard plane. This shifts the standard coordinates of rp by (-0.5,  0.866).
        :param rp: The RobotPosition to be translated
        :param d_point: A Point representing the vector to translate rp in robot coordinates (0 degrees = rp.yaw)
        :return: The RobotPosition representing rp translated by d_point in field coordinates (0 degrees = 0 degrees)
        """
        d_rotated_point_x = d_point.x * cos(rp.yaw) - d_point.y * sin(rp.yaw)
        d_rotated_point_y = d_point.x * sin(rp.yaw) + d_point.y * cos(rp.yaw)
        d_rotated_point = Point(d_rotated_point_x, d_rotated_point_y)

        translated_center = rp.point.shift_by_point(d_rotated_point)

        return RobotPosition(translated_center, rp.yaw)

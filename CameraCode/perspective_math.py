from point import Point
from robot_position import RobotPosition
from math import *

class PerspectiveMath:
    """
    Performs all the math necessary to locate the robot's relative position and yaw from two points from
    vision targets. Points should be at the same height and on the same surface for accuracy. Center-points can be used
    from GRIP to make your life easier. This math will work less and less as the pitch of the camera changes from 0.
    Actually gives the camera pinhole's position and yaw, but it's easy to offset :)
    """

    FOV = 90

    #The distance from the pinhole to the viewing plane, allows for the normalization of calculated points
    K = 1 / tan(FOV / 2)

    #TODO Needs to be calculated. See how_do_work.txt
    pixel_scale = 100

    def calculate_robot_position(self, screen_point1, screen_point2, real_point1, camera_height):
        """
        Calculates the position and yaw of the robot wrt a vision target. Takes in two points at the same height from
        the target and the real location of a point. These points should be translated such that the center of the
        camera's vision is (0, 0), and the top right corner is (resolution_x / 2, resolution_y / 2). Consider's the
        surface the vision target is on as the plane z = 0.
        :param screen_point1: the first point's translated screen position
        :param screen_point2: the second point's translated screen position
        :param real_point1: the real world location of screen_point1 as (x, y) in desired units. Z is defaulted to 0.
        :param camera_height: the height of the camera in the same units as real_point1
        :return: the RobotPosition (x, z, yaw) of the robot wrt the target in the units of real_point1 and degrees
        """

        normalised_point1 = self.__scale_point(screen_point1)
        normalised_point2 = self.__scale_point(screen_point2)

        yaw = self.__calculate_yaw(normalised_point1, normalised_point2)
        location = self.__calculate_camera_location(yaw, camera_height, screen_point1, real_point1)

        return RobotPosition(location, yaw)

    def __scale_point(self, point):
        """
        Scales a point to fit K = cot(FOV / 2) based on self.pixel_scale.
        This point should be translated such that the center of the camera's vision is (0, 0), and the top right corner
        is (resolution_x / 2, resolution_y / 2)

        :param point: the point to normalise
        :return: the normalised point
        """
        return Point(point.x / self.pixel_scale, point.y / self.pixel_scale)

    def __calculate_yaw(self, screen_point1, screen_point2):
        """
        Calculates the yaw rotation of the camera from two vision target points at the same height. These should be
        normalised

        :param screen_point1: the first point
        :param screen_point2: the second point
        :return: the yaw of the camera
        """

        d_y = -self.K * (screen_point1.y - screen_point2.y)
        d_x = screen_point1.x * screen_point2.y - screen_point2.x * screen_point1.y

        return atan2(d_y, d_x)

    def __calculate_camera_location(self, yaw, camera_height, screen_point, real_point):
        """
        Calculates the camera's location (x, z) wrt the vision target
        :param yaw: the calculated yaw of the camera
        :param camera_height: the height of the camera in the desired units
        :param screen_point: a normalised point on the screen
        :param real_point: the real world location (x, y) of screen_point in the same units as camera_height. Z = 0.
        :return: the robot's location (x, z) wrt the vision target in the units of camera_height
        """

        '''
        ┌     ┐   ┌     ┐                          ┌                                ┐
        │ c_x │ = │ a_x │ - (a_y - c_y) / (sp_y) * │ K * sin(yaw) + sp_x * cos(yaw) │
        │ c_z │   │ a_z │                          │ K * cos(yaw) - sp_x * sin(yaw) │
        └     ┘   └     ┘                          └                                ┘
        '''
        x = real_point.x - (real_point.y - camera_height) / screen_point.y * (
                self.K * sin(radians(yaw)) + screen_point.x * cos(radians(yaw)))
        z = real_point.y - (real_point.y - camera_height) / screen_point.y * (
                    self.K * sin(radians(yaw)) + screen_point.x * cos(radians(yaw)))

        return Point(x, z)
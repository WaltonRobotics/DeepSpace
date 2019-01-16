from math import *

class PerspectiveMath:
    """
    Performs all the math necessary to locate the robot's relative position and yaw from two points from
    vision targets. Points should be at the same height and on the same surface for accuracy. Center-points can be used
    from GRIP to make your life easier. This math will work less and less as the pitch of the camera changes from 0.
    Actually gives the camera pinhole's position and yaw, but it's easy to offset :)
    """

    #TODO Verify this angle
    FOV = 90
    #TODO Needs to be calculated. See how_do_work.txt
    pixel_scale = 100
    #TODO This should be how high the camera is in the desired end units
    camera_height = 32
    #TODO This point is the real-world location of screen_point1 (see calculate_robot_position), relative to (0, 0, 0). Z = 0
    real_point = (4, 31)

    #The distance from the pinhole to the viewing plane, allows for the normalization of calculated points
    K = 1 / tan(radians(FOV / 2))

    def calculate_robot_position(self, screen_point1, screen_point2, frame_size):
        """
        Calculates the position and yaw of the robot wrt a vision target. Takes in two points at the same height from
        the target and the real location of a point. Consider's the surface the vision target is on as the plane z = 0.
        :param screen_point1: the first point's (x, y)
        :param screen_point2: the second point's (x, y)
        :param frame_size: the resolution of the frame (vertical, horizontal)
        :return: the RobotPosition ((x, z), yaw) of the robot wrt the target in the units of real_point1 and degrees
        """

        normalised_point1 = self.__scale_point((screen_point1[0] - frame_size[1] / 2, screen_point1[1] - frame_size[0] / 2))
        normalised_point2 = self.__scale_point((screen_point2[0] - frame_size[1] / 2, screen_point2[1] - frame_size[0] / 2))

        yaw = self.__calculate_yaw(normalised_point1, normalised_point2)
        location = self.__calculate_camera_location(yaw, screen_point1)

        return location, yaw

    def __scale_point(self, point):
        """
        Scales a point to fit K = cot(FOV / 2) based on self.pixel_scale.

        :param point: the point to normalise
        :return: the normalised point
        """
        return point[0] / self.pixel_scale, point[1] / self.pixel_scale

    def __calculate_yaw(self, screen_point1, screen_point2):
        """
        Calculates the yaw rotation of the camera from two vision target points at the same height. These should be
        normalised

        :param screen_point1: the first point
        :param screen_point2: the second point
        :return: the yaw of the camera
        """

        d_y = -self.K * (screen_point1[1] - screen_point2[1])
        d_x = screen_point1[0] * screen_point2[1] - screen_point2[0] * screen_point1[1]

        return atan2(d_y, d_x)

    def __calculate_camera_location(self, yaw, screen_point):
        """
        Calculates the camera's location (x, z) wrt the vision target
        :param yaw: the calculated yaw of the camera
        :param screen_point: a normalised point on the screen
        :return: the robot's location (x, z) wrt the vision target in the units of camera_height
        """

        '''
        ┌     ┐   ┌     ┐                          ┌                                ┐
        │ c_x │ = │ a_x │ - (a_y - c_y) / (sp_y) * │ K * sin(yaw) + sp_x * cos(yaw) │
        │ c_z │   │ a_z │                          │ K * cos(yaw) - sp_x * sin(yaw) │
        └     ┘   └     ┘                          └                                ┘
        '''
        x = self.real_point[0] - (self.real_point[1] - self.camera_height) / screen_point[1] * (
                self.K * sin(radians(yaw)) + screen_point[0] * cos(radians(yaw)))
        z = self.real_point[1] - (self.real_point[1] - self.camera_height) / screen_point[1] * (
                    self.K * sin(radians(yaw)) + screen_point[0] * cos(radians(yaw)))

        return x, z
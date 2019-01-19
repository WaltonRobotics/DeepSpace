import itertools as it
import math
import cv2
import numpy as np
from math import *
from networktables import NetworkTables
import json
import time
import sys
from cscore import CameraServer, VideoSource

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []

"""Report parse error."""
def parseError(str):
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

"""Read single camera configuration."""
def readCameraConfig(config):
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    cam.config = config

    cameraConfigs.append(cam)
    return True

"""Read configuration file."""
def readConfig():
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    return True

"""Start running the camera."""
def startCamera(config):
    print("Starting camera '{}' on {}".format(config.name, config.path))
    camera = CameraServer.getInstance() \
        .startAutomaticCapture(name=config.name, path=config.path)

    camera.setConfigJson(json.dumps(config.config))

    return camera

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)


    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)

    # start cameras
    cameras = []
    for cameraConfig in cameraConfigs:
        cameras.append(startCamera(cameraConfig))


colors = [
    (0, 0, 255),
    (0, 255, 0),
]


def pairwise(iterable):
    a = iter(iterable)
    return it.zip_longest(a, a, fillvalue=None)


class Target:

    def __init__(self, left_rect, right_rect): 
        self.left_rect = left_rect
        self.right_rect = right_rect

    @property
    def average_center(self):
        return (self.left_rect[0] + self.right_rect[0]) / 2

    @property
    def average_x(self):
        return (self.right_rect[0][0] + self.left_rect[0][0]) / 2

    @property
    def average_y(self):
        return (self.left_rect[0][1] + self.right_rect[0][1]) / 2

    @property
    def center_point_left(self):
        return self.left_rect[0]

    @property
    def center_point_right(self):
        return self.right_rect[0]

    @property
    def angle_left(self):
        return self.left_rect[2]

    @property
    def angle_right(self):
        return self.right_rect[2]

    def draw_target(self, frame):
        box_l = cv2.boxPoints(self.left_rect)
        box_r = cv2.boxPoints(self.right_rect)
        box_l = np.int0(box_l)
        box_r = np.int0(box_r)

        cv2.drawContours(frame, [box_l], 0, colors[0], 2)
        cv2.drawContours(frame, [box_r], 0, colors[1], 2)


class ContourTracker:

    def __init__(self):
        pass

    def distinguish_contours(self):
        pass

    def __get_min_area_rects(self, contours):
        contour_rects = []

        for contour in contours:
            rect = cv2.minAreaRect(contour)
            contour_rects.append(rect)

        return contour_rects

    def find_closest_contour(self, contours, frame_size):

        contour_rects = self.__get_min_area_rects(contours)

        def get_x(rect):
            return rect[0][0]

        contour_rects.sort(key=get_x)

        def is_right(rect):
            return rect[2] > -45

        def is_left(rect):
            return rect[2] <= -45

        """

        Check if the contour is facing right

        """

        if contour_rects and is_right(contour_rects[0]):
            first = [Target(None, contour_rects.pop(0))]

        else:
            first = []

        """

        Check if the contour is facing left

        """
        if contour_rects and is_left(contour_rects[-1]):
            last = [Target(contour_rects.pop(-1), None)]

        else:
            last = []

        remainder = []

        for left, right in pairwise(contour_rects):
            remainder.append(Target(left, right))

        if len(remainder) > 0:
            remainder = min(remainder, key=lambda target: math.fabs(target.average_x - frame_size[1] / 2))
        else:
            remainder = None

        return remainder


class FilterLines:
    """
    An OpenCV pipeline generated by GRIP.
    """

    def __init__(self):
        """initializes all values to presets or None if need to be set
        """

        self.__hsl_threshold_hue = [69.60431654676259, 99.01528013582343]
        self.__hsl_threshold_saturation = [155.93525179856115, 255.0]
        self.__hsl_threshold_luminance = [91.72661870503596, 155.42444821731752]

        self.hsl_threshold_output = None

        self.__find_contours_input = self.hsl_threshold_output
        self.__find_contours_external_only = False

        self.find_contours_output = None

        self.__filter_contours_contours = self.find_contours_output
        self.__filter_contours_min_area = 6.0
        self.__filter_contours_min_perimeter = 5.0
        self.__filter_contours_min_width = 4.0
        self.__filter_contours_max_width = 1000.0
        self.__filter_contours_min_height = 3.0
        self.__filter_contours_max_height = 1000.0
        self.__filter_contours_solidity = [0, 100]
        self.__filter_contours_max_vertices = 1000000.0
        self.__filter_contours_min_vertices = 4.0
        self.__filter_contours_min_ratio = 0.0
        self.__filter_contours_max_ratio = 1000.0

        self.filter_contours_output = None

        self.get_mat_info_size = None

        self.get_mat_info_empty = None

        self.get_mat_info_channels = None

        self.get_mat_info_cols = None

        self.get_mat_info_rows = None

        self.get_mat_info_high_value = None

    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step HSL_Threshold0:
        self.__hsl_threshold_input = source0
        (self.hsl_threshold_output) = self.__hsl_threshold(self.__hsl_threshold_input, self.__hsl_threshold_hue,
                                                           self.__hsl_threshold_saturation,
                                                           self.__hsl_threshold_luminance)

        # Step Find_Contours0:
        self.__find_contours_input = self.hsl_threshold_output
        (self.find_contours_output) = self.__find_contours(self.__find_contours_input,
                                                           self.__find_contours_external_only)

        # Step Filter_Contours0:
        self.__filter_contours_contours = self.find_contours_output
        (self.filter_contours_output) = self.__filter_contours(self.__filter_contours_contours,
                                                               self.__filter_contours_min_area,
                                                               self.__filter_contours_min_perimeter,
                                                               self.__filter_contours_min_width,
                                                               self.__filter_contours_max_width,
                                                               self.__filter_contours_min_height,
                                                               self.__filter_contours_max_height,
                                                               self.__filter_contours_solidity,
                                                               self.__filter_contours_max_vertices,
                                                               self.__filter_contours_min_vertices,
                                                               self.__filter_contours_min_ratio,
                                                               self.__filter_contours_max_ratio)

        # Step Get_Mat_Info0:
        self.__get_mat_info_input = source0
        (self.get_mat_info_size, self.get_mat_info_empty, self.get_mat_info_channels, self.get_mat_info_cols,
         self.get_mat_info_rows) = self.__get_mat_info(self.__get_mat_info_input)

    @staticmethod
    def __hsl_threshold(input, hue, sat, lum):
        """Segment an image based on hue, saturation, and luminance ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max luminance.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HLS)
        return cv2.inRange(out, (hue[0], lum[0], sat[0]), (hue[1], lum[1], sat[1]))

    @staticmethod
    def __find_contours(input, external_only):
        """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
        Args:
            input: A numpy.ndarray.
            external_only: A boolean. If true only external contours are found.
        Return:
            A list of numpy.ndarray where each one represents a contour.
        """
        if (external_only):
            mode = cv2.RETR_EXTERNAL
        else:
            mode = cv2.RETR_LIST
        method = cv2.CHAIN_APPROX_SIMPLE
        im2, contours, hierarchy = cv2.findContours(input, mode=mode, method=method)
        return contours

    @staticmethod
    def __filter_contours(input_contours, min_area, min_perimeter, min_width, max_width,
                          min_height, max_height, solidity, max_vertex_count, min_vertex_count,
                          min_ratio, max_ratio):
        """Filters out contours that do not meet certain criteria.
        Args:
            input_contours: Contours as a list of numpy.ndarray.
            min_area: The minimum area of a contour that will be kept.
            min_perimeter: The minimum perimeter of a contour that will be kept.
            min_width: Minimum width of a contour.
            max_width: MaxWidth maximum width.
            min_height: Minimum height.
            max_height: Maximimum height.
            solidity: The minimum and maximum solidity of a contour.
            min_vertex_count: Minimum vertex Count of the contours.
            max_vertex_count: Maximum vertex Count.
            min_ratio: Minimum ratio of width to height.
            max_ratio: Maximum ratio of width to height.
        Returns:
            Contours as a list of numpy.ndarray.
        """
        output = []
        for contour in input_contours:
            x, y, w, h = cv2.boundingRect(contour)
            if (w < min_width or w > max_width):
                continue
            if (h < min_height or h > max_height):
                continue
            area = cv2.contourArea(contour)
            if (area < min_area):
                continue
            if (cv2.arcLength(contour, True) < min_perimeter):
                continue
            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            if (solid < solidity[0] or solid > solidity[1]):
                continue
            if (len(contour) < min_vertex_count or len(contour) > max_vertex_count):
                continue
            ratio = (float)(w) / h
            if (ratio < min_ratio or ratio > max_ratio):
                continue
            output.append(contour)
        return output

    @staticmethod
    def __get_mat_info(src):
        """Gets information about given Mat.
        Args:
            src: A numpy.ndarray.
        Returns:
            The size of the mat as a list of two numbers.
            A boolean that is true if the mat is empty.
            The number of the channels in the mat.
            The number of columns.
            The number of rows.
            The highest value in the mat.
        """
        cols, rows, channels = src.shape
        empty = (src.size == 0)

        mat_size = (rows, cols)
        return mat_size, empty, channels, cols, rows
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
    #TODO This represents the vector between the camera and the center of the robot
    camera_shift = (0, 0)

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
        location = self.__shift_location(location, yaw)

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

    def __shift_location(self, location, yaw):
        offset_x = self.camera_shift[0] * cos(yaw) - self.camera_shift[1] * sin(yaw)
        offset_y = self.camera_shift[1] * cos(yaw) + self.camera_shift[0] * sin(yaw)
        return location[0] + offset_x, location[1] + offset_y


if __name__ == "__main__":
    camera = cv2.VideoCapture(0)

    # img = "./vision examples/CargoAngledLine48in.jpg"
    # img = "./vision examples/CargoAngledDark48in.jpg"
    # source = cv2.imread(img)

    grip = FilterLines()
    # video = cv2.VideoCapture(convert)

    my_processor = ContourTracker()
    perspective_math = PerspectiveMath()

    NetworkTables.initialize(server='roborio-2974-frc.local')
    sd = NetworkTables.getTable('SmartDashboard')

    while True:

        ret, source = camera.read()
        grip.process(source)
        frame_size = grip.get_mat_info_size

        center_target = my_processor.find_closest_contour(grip.filter_contours_output, frame_size)

        if center_target is not None:
            robot_pose = perspective_math.calculate_robot_position(center_target.center_point_left,
                                                                   center_target.center_point_left,
                                                                   frame_size)
            print("(%s, %s), %s degrees" % (robot_pose[0][0], robot_pose[0][1], robot_pose[1]))
            sd.putNumber('x_value', robot_pose[0][0])
            sd.putNumber('y_value', robot_pose[0][1])
            sd.putNumber('angle', robot_pose[1])
        else:
            print("There are no contours found")

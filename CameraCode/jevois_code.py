import itertools as it
import math  # for cos, sin, etc
import cv2
import libjevois as jevois
import numpy as np
from datetime import datetime as date


def distance_2d(point0, point1):
    """
    :param point0:
    :param point1:
    :return: the distance between point0 and point1
    """
    return math.sqrt(math.pow(point0[0] - point1[0], 2) + math.pow(point0[1] - point1[1], 2))

def order_corners_clockwise(tape):
        """
        Sorts the corners of a contour clockwise from the top left point
        :param tape: the contour to sort the corners of
        :return: the sorted corners
        """
        # get corners of contour
        corners = tape.box_points
        # sort corners by x value
        x_sorted = corners[np.argsort(corners[:, 0]), :]

        # grab the left-most and right-most points from the sorted x-coordinate points
        left_most = x_sorted[:2, :]
        right_most = x_sorted[2:, :]

        # now, sort the left-most coordinates according to their y-coordinates so we can grab the top-left and
        # bottom-left points, respectively
        left_most = left_most[np.argsort(left_most[:, 1]), :]
        (tl, bl) = left_most

        # now that we have the top-left coordinate, use it as an anchor to calculate the Euclidean distance between the
        # top-left and right-most points; by the Pythagorean theorem, the point with the largest distance will be our
        # bottom-right point
        if distance_2d(tl, right_most[0]) > distance_2d(tl, right_most[1]):
            br, tr = right_most
        else:
            tr, br = right_most

        # return the coordinates in top-left, top-right, bottom-right, and bottom-left order
        return np.array([tl, tr, br, bl], dtype=np.float32)

def pairwise(iterable):
    a = iter(iterable)
    return it.zip_longest(a, a, fillvalue=None)

class Pose:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle = 0

class FirstPython:
    def __init__(self):
        # HSV color range to use:
        #
        # H: 0=red/do not use because of wraparound, 30=yellow, 45=light green, 60=green, 75=green cyan, 90=cyan,
        #      105=light blue, 120=blue, 135=purple, 150=pink
        self.HLSmin = np.array([50, 100, 180], dtype=np.uint8)
        self.HLSmax = np.array([70, 255, 255], dtype=np.uint8)
        self.min_area = 200.0
        self.min_perimeter = 10.0
        self.min_width = 5.0
        self.max_width = 1000.0
        self.min_height = 0.0
        self.max_height = 1000.0
        self.solidity = [0, 100]
        self.max_vertices = 1000000.0
        self.min_vertices = 4.0
        self.min_ratio = 0.5
        self.max_ratio = 1.1
        self.max_contour_y = 50

        self.object_points = np.array(
            [[-5.936, 31.5, 0.0], [-4.0, 31, 0.0], [-5.375, 25.675, 0.0], [-7.316, 26.175, 0.0],  # Left points
             [4.0, 31, 0.0], [5.936, 31.5, 0.0], [7.316, 26.175, 0.0], [5.375, 25.675, 0.0]], dtype=np.float32)  # Right points

        self.decision_tolerance = 0.05
        self.current_target = np.array([None, None])
        self.target_lost_factor = 1.25
        self.target_lost_time = -1
        self.target_lost_timeout = 5


        self.isEnabled = False

    def load_camera_calibration(self, w, h):
        """
        Load the camera's calibration matrices, determined by the width and height of the image
        :param w:
        :param h:
        :return:
        """
        cpf = "/jevois/share/camera/calibration{}x{}.yaml".format(w, h)
        fs = cv2.FileStorage(cpf, cv2.FILE_STORAGE_READ)
        if fs.isOpened():
            self.cam_matrix = fs.getNode("camera_matrix").mat()
            self.dist_coeffs = fs.getNode("distortion_coefficients").mat()
            #jevois.LINFO("Loaded camera calibration from {}".format(cpf))
        else:
            jevois.LFATAL("Failed to read camera parameters from file [{}]".format(cpf))

    def detect(self, imgbgr):
        """

        :param imgbgr:
        :return: binary image and tape contours sorted left to right
        """

        if not hasattr(self, 'erodeElement'):
            self.erode_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            self.dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))

        #Loads intrinsic parameters of the camera while we're at it
        res_y, res_x, channels = imgbgr.shape
        if not hasattr(self, 'camMatrix'): self.load_camera_calibration(res_x, res_y)

        # Convert input image to HSV:
        imghsv = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2HLS)

        # Isolate pixels inside our desired HSV range:
        imgth = cv2.inRange(imghsv, self.HLSmin, self.HLSmax)

        # Apply morphological operations to cleanup the image noise:
        imgth = cv2.erode(imgth, self.erode_element)
        imgth = cv2.dilate(imgth, self.dilate_element)

        # Detect objects by finding contours:
        contours, hierarchy = cv2.findContours(imgth, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Filter Contours by...
        output = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            #Contour height
            if y < self.max_contour_y:
                continue
            # Width
            if w < self.min_width or w > self.max_width:
                continue
            # Height
            if h < self.min_height or h > self.max_height:
                continue
            area = cv2.contourArea(contour)
            # Area
            if area < self.min_area:
                continue
            # Perimeter
            if cv2.arcLength(contour, True) < self.min_perimeter:
                continue
            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            # Solidity
            if solid < self.solidity[0] or solid > self.solidity[1]:
                continue
            # Vertex Count
            if len(contour) < self.min_vertices or len(contour) > self.max_vertices:
                continue
            length, width = cv2.minAreaRect(contour)[1]
            ratio = area / (length * width)
            # areas ratio
            if ratio < self.min_ratio or ratio > self.max_ratio:
                continue
            output.append(TapePiece(hull))

        # Sort the tapes from left to right
        sorted_output = sorted(output, key=lambda tape: tape.center_point[0])

        return imgth, sorted_output

    @staticmethod
    def draw_image(outimg, inimg):

        if outimg.valid():
            jevois.paste(inimg, outimg, 0, 0)

    @staticmethod
    def draw_tapes(outimg, tapes):
        # Display any results requested by the users:
        if outimg.valid():
            for tape in tapes:
                tape.draw(outimg)

    @staticmethod
    def draw_targets(outimg, targets):
        if outimg.valid():
            for target in targets:
                target.draw_quadrilateral(outimg)

    def estimate_poses(self, targets):
        """
        Estimates the rotation and position of the camera wrt the target
        :param targets: An array of targets
        :return: rotation and translation of the camera
        """

        rotation_array = []
        translation_array = []

        for target in targets:
            image_points_left = np.array(order_corners_clockwise(target.left_tape))

            image_points_right = np.array(order_corners_clockwise(target.right_tape))

            image_points = np.concatenate((image_points_left, image_points_right))
            ret, rotation, translation = cv2.solvePnP(self.object_points, image_points, self.cam_matrix,
                                                      self.dist_coeffs)
            rotation_array.append(rotation)
            translation_array.append(translation)
        return rotation_array, translation_array

    def estimate_pose(self, target):
        """
        Estimates the rotation and position of the camera wrt the target
        :param target: An array of targets
        :return: rotation and translation of the camera
        """
        rotation, translation = self.estimate_poses([target])
        return rotation[0], translation[0]

    def send_all_serial(self, targets):
        """
        Sends a message over the SerialPort to the RoboRIO
        :param targets:
        :return:
        """
        if self.current_target[0] is not None and len(self.current_target[0]) is 3:
            current_target, rotation, translation = self.current_target[0]
            pose = Pose()
            pose.x = translation[2]
            pose.y = translation[0]
            pose.z = translation[1]
            pose.angle = -rotation[2]

            x_key = 'x' if pose.x < 0 else 'X'
            x = min(int(round(abs(float(pose.x)) * 2.54)), 999)

            y_key = 'z' if pose.z < 0 else 'Y'
            y = min(int(round(abs(float(pose.y)) * 2.54)), 999)

            z_key = 'z' if pose.z < 0 else 'Z'
            z = min(int(round(abs(float(pose.z)) * 2.54)), 99)

            angle_key = 'a' if pose.angle < 0 else 'A'

            angle = min(int(round(math.degrees(float(pose.angle)))), 999)

            jevois.sendSerial(
                "{0:s}{1:3d}{2:s}{3:3d}{4:s}{5:2d}{6:s}{7:3d}N{8:1d}".format(x_key, x, y_key, y, z_key, z, angle_key,
                                                                             angle, len(targets)))  # pose

        else:
            jevois.sendSerial("FN{}".format(min(len(targets), 9)))

    def processNoUSB(self, inframe):
        """
        Process the inframe without a USB connection to JeVois Inventor. This is run automatically by the camera.
        :param inframe:
        :return:
        """
        # Get the next camera image (may block until it is captured). To avoid wasting much time assembling a composite
        # output image with multiple panels by concatenating numpy arrays, in this module we use raw YUYV images and
        # fast paste and draw operations provided by JeVois on those images:
        inimg = inframe.get()

        # target_tracker = TargetTracker()

        imgbgr = jevois.convertToCvBGR(inimg)
        inframe.done()

        # Get a list of quadrilateral convex hulls for all good objects:
        imgth, tapes = self.detect(imgbgr)

        targets = self.define_targets(tapes)

        if self.current_target[1] is None:
            self.choose_target(targets)

        if self.current_target[1] is not None:
            self.track(targets)

        self.send_all_serial(targets)


    def process(self, inframe, outframe):
        """
        Process the inframe with USB connection to JeVois Inventor. This is run automatically by the camera.
        :param inframe:
        :param outframe:
        :return:
        """
        # Get the next camera image (may block until it is captured). To avoid wasting much time assembling a composite
        # output image with multiple panels by concatenating numpy arrays, in this module we use raw YUYV images and
        # fast paste and draw operations provided by JeVois on those images:
        inimg = inframe.get()
        outimg = outframe.get()
        outimg.require("output", 640, 480, jevois.V4L2_PIX_FMT_YUYV)

        # target_tracker = TargetTracker()

        self.draw_image(outimg, inimg)
        imgbgr = jevois.convertToCvBGR(inimg)
        inframe.done()

        # Get a list of quadrilateral convex hulls for all good objects:
        imgth, tapes = self.detect(imgbgr)
        self.draw_tapes(outimg, tapes)


        targets = self.define_targets(tapes)
        self.draw_targets(outimg, targets)

        if self.current_target[1] is None:
            self.choose_target(targets, outimg)

        if self.current_target[1] is not None:
            self.track(targets, outimg)

        self.send_all_serial(targets)

        outframe.send()

    @staticmethod
    def percent_difference(value1, value2):
        """
        Finds the percent difference between two values, between 0.0 and 1.0
        :param value1:
        :param value2:
        :return:
        """
        return math.fabs(value1 - value2) / (value1 + value2)

    @staticmethod
    def distance_from_origin(target_data):
        """
        :param target_data:
        :return: the distance between point and (0, y, 0)
        """
        return math.sqrt(math.pow(target_data[2][0], 2) + math.pow(target_data[2][2], 2))

    @staticmethod
    def define_targets(tapes):
        targets = []
        if len(tapes) >= 2:
            while len(tapes) > 0 and tapes[0].parity is 'right':
                tapes.pop(0)
            while len(tapes) > 0 and tapes[-1].parity is 'left':
                tapes.pop(-1)

            if len(tapes) > 0:
                first_tape = tapes[0]
                i = 1
                while i < len(tapes) - 1:
                    second_tape = tapes[i]
                    if first_tape.parity is 'left' and second_tape.parity is 'right':
                        targets.append(Target(first_tape, second_tape))
                        i += 1
                    first_tape = tapes[i]
                    i += 1
                if i is len(tapes) - 1:
                    second_tape = tapes[i]
                    if first_tape.parity is 'left' and second_tape.parity is 'right':
                        targets.append(Target(first_tape, second_tape))
        return targets

    def choose_target(self, targets, outimg=None):
        """
        Chooses a target closest to the camera. If multiple targets are almost the same distance from the camera, return None
        :param targets:
        :param outimg:
        :return:
        """
        target_data = []
        if len(targets) > 0:
            rotations, translations = self.estimate_poses(targets)
            for i, target in enumerate(targets):
                target_data.append((target, rotations[i], translations[i]))
            target_data.sort(key=self.distance_from_origin)

            if len(target_data) != 1 and self.percent_difference(self.distance_from_origin(target_data[0]),
                                                                 self.distance_from_origin(target_data[1])) <= \
                    self.decision_tolerance:
                jevois.LINFO("Cannot decide which target to choose")
                if outimg is not None:
                    jevois.writeText(outimg, "Cannot decide which target to choose", 3, 3, jevois.YUYV.White,
                                     jevois.Font.Font6x10)
            else:
                closest_target, rotation, translation = target_data[0]
                # tracker = cv2.TrackerKCF_create()
                # tracker.init(inimg, closest_target.bounding_rectangle)
                closest_box = closest_target.bounding_rectangle
                self.current_target = np.array([target_data, closest_box])
                jevois.LINFO("Target found")

    def track(self, targets, outimg=None):
        """
        Updates tracking on the currently selected target
        :param targets:
        :param outimg:
        :return:
        """
        success, target = self.find_target_closest_to_current(targets)
        if success:
            self.target_lost_time = -1
            rotation, translation = self.estimate_pose(target)
            self.current_target[0] = (target, rotation, translation)
            self.current_target[1] = target.bounding_rectangle
            if outimg is not None:
                target.draw_quadrilateral(outimg, 0x048f)
                rstring = "Rotation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(math.degrees(rotation[0]), math.degrees(
                    rotation[1]), math.degrees(rotation[2]))
                jevois.writeText(outimg, rstring, 3, 3, jevois.YUYV.White, jevois.Font.Font6x10)
                tstring = "Translation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(float(translation[0]),
                                                                                float(translation[1]),
                                                                                float(translation[2]))
                jevois.writeText(outimg, tstring, 3, 15, jevois.YUYV.White, jevois.Font.Font6x10)
        else:
            self.current_target[0] = None
            if self.target_lost_time is -1:
                self.target_lost_time = date.now().second
            if date.now().second - self.target_lost_time < self.target_lost_timeout:
                if outimg is not None:
                    jevois.writeText(outimg, "Target lost, waiting {} more seconds".format(self.target_lost_timeout -
                                     date.now().second + self.target_lost_time), 3, 3, jevois.YUYV.White,
                                     jevois.Font.Font6x10)
                jevois.LINFO("Target lost, waiting {} more seconds".format(self.target_lost_timeout -
                                     date.now().second + self.target_lost_time))
            else:
                self.current_target[1] = None
                self.target_lost_time = -1
                jevois.writeText(outimg, "Lost target has been timed out", 3, 3, jevois.YUYV.White,
                                 jevois.Font.Font6x10)
                jevois.LINFO("Lost target has been timed out")

    def find_target_closest_to_current(self, targets):
        """
        Finds the target in targets closest to fitting inside of rectangle
        :param targets:
        :return:
        """
        if len(targets) is 0:
            return False, None
        targets.sort(key=lambda target: distance_2d(self.current_target[1], target.bounding_rectangle))
        most_likely_target = targets[0]
        if distance_2d(most_likely_target.bounding_rectangle, self.current_target[1]) > \
                most_likely_target.bounding_rectangle[3] * self.target_lost_factor:
            return False, None
        return True, targets[0]

class Target:

    def __init__(self, left_tape, right_tape):
        self.left_tape = left_tape
        self.right_tape = right_tape

    @property
    def average_center(self):
        return Target.average_x, Target.average_y

    @property
    def average_x(self):
        return (self.right_tape[0][0] + self.left_tape[0][0]) / 2

    @property
    def average_y(self):
        return (self.left_tape[0][1] + self.right_tape[0][1]) / 2

    @property
    def bounding_quadrilateral(self):
        bl = np.array([math.inf, -math.inf])
        tl = np.array([math.inf, math.inf])
        for point in self.left_tape.box_points:
            bl[0] = min(float(bl[0]), float(point[0]))
            bl[1] = max(float(bl[1]), float(point[1]))
            tl[0] = min(float(tl[0]), float(point[0]))
            tl[1] = min(float(tl[1]), float(point[1]))

        br = np.array([-math.inf, -math.inf])
        tr = np.array([-math.inf, math.inf])
        for point in self.right_tape.box_points:
            br[0] = max(float(br[0]), float(point[0]))
            br[1] = max(float(br[1]), float(point[1]))
            tr[0] = max(float(tr[0]), float(point[0]))
            tr[1] = min(float(tr[1]), float(point[1]))
        return (tl[0], tl[1]), (tr[0], tr[1]), (br[0], br[1]), (bl[0], bl[1])

    @property
    def bounding_rectangle(self):
        (x, y), tr, br, bl = self.bounding_quadrilateral
        w = max(br[0], tr[0]) - x
        h = max(br[1], bl[1]) - y
        return x, y, w, h

    def draw_quadrilateral(self, outimg, color=jevois.YUYV.LightPurple):
        if self.left_tape is not None and self.right_tape is not None:
            bbox = self.bounding_quadrilateral
            point0 = bbox[-1]
            for point1 in bbox:
                jevois.drawLine(outimg, int(point0[0]), int(point0[1]), int(point1[0]), int(point1[1]), 1, color)
                point0 = point1

class TapePiece:
    def __init__(self, contour):
        self.contour = contour

    @property
    def angle(self):
        return cv2.minAreaRect(self.contour)[2]

    @property
    def center_point(self):
        moments = cv2.moments(self.contour)
        return moments['m10'] / moments['m00'], moments['m01'] / moments['m00']

    @property
    def parity(self):
        corners = order_corners_clockwise(self)
        if corners[0][0] > corners[3][0] or corners[1][0] > corners[2][0]:
            return 'left'
        elif corners[3][0] > corners[0][0] or corners[2][0] > corners[1][0]:
            #If the bottom left corner is more right than the top left
            return 'right'
        else:
            return ''

    @property
    def box_points(self):
        return cv2.boxPoints(cv2.minAreaRect(self.contour))

    def draw(self, outimg):
        if self.contour.size > 0:
            point0 = self.contour[-1]

            for point1 in self.contour:
                jevois.drawLine(outimg, int(point0[0, 0]), int(point0[0, 1]),
                                int(point1[0, 0]), int(point1[0, 1]), 1, jevois.YUYV.MedGrey)

                point0 = point1
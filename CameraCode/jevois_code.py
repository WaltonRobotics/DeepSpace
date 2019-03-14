import math  # for cos, sin, etc
from datetime import datetime as date

import cv2
import libjevois as jevois
import numpy as np


def distance_2d(point0, point1):
    """
    :param point0:
    :param point1:
    :return: the distance between point0 and point1
    """
    return math.sqrt(math.pow(point0[0] - point1[0], 2) + math.pow(point0[1] - point1[1], 2))


def distance_3d(point0, point1):
    """
    :param point0:
    :param point1:
    :return: the distance between point0 and point1
    """
    return math.sqrt(math.pow(point0[0] - point1[0], 2) + math.pow(point0[1] - point1[1], 2) + math.pow(point0[2] -
                                                                                                        point1[2], 2))


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

    # sort the left-most coordinates in the same fashion
    right_most = right_most[np.argsort(right_most[:, 1]), :]
    (tr, br) = right_most

    # return the coordinates in top-left, top-right, bottom-right, and bottom-left order
    return np.array([tl, tr, br, bl], dtype=np.float32)


def define_tape_corners(angle, length):
    """
    Define the vision target points from the length of the tape and the angle, to account for inconsistencies.
    Assume the width of the tape is 2 inches, the top point is 31.5 inches high, and the distance between tapes is 8
    inches at the closest point.
    :param angle:
    :param length:
    :return:
    """
    tl_x = 4
    tr_y = 31.5

    tl_y = tr_y - 2 * math.sin(angle)

    tr_x = tl_x + 2 * math.cos(angle)

    br_x = tr_x + length * math.sin(angle)
    br_y = tr_y - length * math.cos(angle)

    bl_x = tl_x + length * math.sin(angle)
    bl_y = tl_y - length * math.cos(angle)

    return np.array([[-tr_x, tr_y, 0.0], [-tl_x, tl_y, 0.0], [-bl_x, bl_y, 0.0], [-br_x, br_y, 0.0],
                     [tl_x, tl_y, 0.0], [tr_x, tr_y, 0.0], [br_x, br_y, 0.0], [bl_x, bl_y, 0.0]], dtype=np.float32)


def euler_to_matrix(euler):
    """
    I found this on the internet\n
    http://answers.opencv.org/question/88531/how-can-i-get-rotation-vector-from-euler-angles/ \n
    :param euler: the euler angles to convert
    :return:
    """
    x = euler[0]
    y = euler[1]
    z = euler[2]

    ch = math.cos(z)
    sh = math.sin(z)
    ca = math.cos(y)
    sa = math.sin(y)
    cb = math.cos(x)
    sb = math.sin(x)

    m00 = ch * ca
    m01 = sh * sb - ch * sa * cb
    m02 = ch * sa * sb + sh * cb
    m10 = sa
    m11 = ca * cb
    m12 = -ca * sb
    m20 = -sh * ca
    m21 = sh * sa * cb + ch * sb
    m22 = -sh * sa * sb + ch * cb

    return np.array([[m00, m01, m02],[m10, m11, m12], [m20, m21, m22]])

class Pose:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle = 0


class HiRezDeepSpace:
    def __init__(self):
        # !!THERE CAN BE NO FILE LOADING IN THE __init__ FUNCTION!!

        # Values used in detect
        self.HLSmin = np.array([55, 100, 180], dtype=np.uint8)
        self.HLSmax = np.array([70, 255, 255], dtype=np.uint8)
        self.min_area = 150.0
        self.min_perimeter = 10.0
        self.min_width = 0.0
        self.max_width = 1000.0
        self.min_height = 0.0
        self.max_height = 1000.0
        self.solidity = [0, 100]
        self.max_vertices = 1000000.0
        self.min_vertices = 4.0
        self.min_ratio = 0.5
        self.max_ratio = 1.1
        self.min_h_w_ratio = 1.0
        self.max_contour_y = 0
        self.targets = []

        # Values used in estimate_poses
        tape_angle = math.asin(0.25)
        # tape_angle = math.radians(10)
        tape_length = 5.5
        self.object_points = define_tape_corners(tape_angle, tape_length)
        self.offset_x = 2
        self.offset_z = 0
        euler_assumption = np.array([math.radians(50), 0.0, 0.0], dtype=np.float32)
        self.rotation_assumption = cv2.Rodrigues(euler_to_matrix(euler_assumption))[0]
        self.translation_assumption = np.array([0, 50, 0], dtype=np.float32).reshape(3, 1)

        # Values used to filter out unlikely targets
        self.decision_tolerance = 0.05
        self.current_target = np.array([None, None])
        self.target_lost_factor = 1.25 #Larger means it will choose a new target sooner
        self.target_lost_time = -1
        self.target_lost_timeout = 0.25
        self.min_x_ang = -180
        self.max_x_ang = 180
        self.min_z_ang = -20
        self.max_z_ang = 20
        self.min_y = 0

        # Values used with user-commands
        self.enabled = True
        self.save_frame = 0
        self.current_time = date.now()


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
            # jevois.LINFO("Loaded camera calibration from {}".format(cpf))
        else:
            jevois.LFATAL("Failed to read camera parameters from file [{}]".format(cpf))

    def detect(self, imgbgr):
        """
        Filters out information that are not tape pieces
        :param imgbgr:
        :return: binary image and tape contours sorted left to right
        """

        if not hasattr(self, 'erodeElement'):
            self.erode_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            self.dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))

        # Loads intrinsic parameters of the camera while we're at it
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
            # Contour height
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
            if h / w < self.min_h_w_ratio:
                continue
            output.append(TapePiece(hull))

        # Sort the tapes from left to right
        sorted_output = sorted(output, key=lambda tape: tape.center_point[0])

        return imgth, sorted_output

    @staticmethod
    def draw_image(outimg, inimg):
        """
        Draws inimg onto outimg
        :param outimg:
        :param inimg:
        :return:
        """
        if outimg.valid():
            jevois.paste(inimg, outimg, 0, 0)

    @staticmethod
    def draw_tapes(outimg, tapes):
        """
        Draws boxes around all of the tape pieces
        :param outimg:
        :param tapes:
        :return:
        """
        if outimg.valid():
            for tape in tapes:
                tape.draw(outimg)

    @staticmethod
    def draw_targets(outimg, targets):
        """
        Draws boxes around all of the targets
        :param outimg:
        :param targets:
        :return:
        """
        if outimg.valid():
            for target in targets:
                target.draw_trapezoid(outimg)


    def estimate_poses(self, targets):
        """
        Estimates the rotation and position of the camera wrt all the targets\n\n
        See also:\n
        https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#solvepnp\n
        https://stackoverflow.com/questions/14515200/python-opencv-solvepnp-yields-wrong-translation-vector
        :param targets: An array of targets
        :return: rotation and translation of the camera
        """

        rotation_array = []
        translation_array = []

        for target in targets:
            # Setup point correspondences
            image_points = target.tape_corners

            # Collect the 3 rotation vectors and origin translation vector
            ret, rvecs, tvecs = cv2.solvePnP(self.object_points, image_points, self.cam_matrix, self.dist_coeffs,
                                             self.rotation_assumption, self.translation_assumption, False)

            # Calculate the actual camera position
            np_rodrigues = np.asarray(rvecs[:, :], np.float64)
            rot_matrix = cv2.Rodrigues(np_rodrigues)[0]
            translation = -np.matrix(rot_matrix).T * np.matrix(tvecs)

            # Calculate the Euler angles (in degrees)
            projection_matrix = np.hstack((rot_matrix, tvecs))
            rotation = cv2.decomposeProjectionMatrix(projection_matrix)[-1]

            rotation_array.append(rotation)
            translation_array.append(translation)
        return rotation_array, translation_array

    def estimate_pose(self, target):
        """
        Estimates the rotation and position of the camera wrt one target
        :param target: An array of targets
        :return: rotation and translation of the camera
        """
        rotation, translation = self.estimate_poses([target])
        return rotation[0], translation[0]

    def send_all_serial(self):
        """
        Sends a message over the SerialPort to the RoboRIO
        :return:
        """
        if self.current_target[0] is not None and len(self.current_target[0]) is 3:
            current_target, rotation, translation = self.current_target[0]
            pose = Pose()
            pose.x = -(translation[2] + self.offset_z)
            pose.y = -(translation[0] + self.offset_x)
            pose.z = translation[1]
            pose.angle = rotation[1]

            x_key = 'x' if pose.x < 0 else 'X'
            x = min(int(round(abs(float(pose.x)) * 2.54)), 999)

            y_key = 'y' if pose.y < 0 else 'Y'
            y = min(int(round(abs(float(pose.y)) * 2.54)), 999)

            z_key = 'z' if pose.z < 0 else 'Z'
            z = min(int(round(abs(float(pose.z)) * 2.54)), 999)

            angle_key = 'a' if pose.angle < 0 else 'A'

            angle = min(int(round(abs(float(pose.angle)))), 999)

            jevois.sendSerial(
                "{0:s}{1:03d}{2:s}{3:03d}{4:s}{5:03d}{6:s}{7:03d}N{8:1d}".format(x_key, x, y_key, y, z_key, z,
                                                                                 angle_key, angle, len(self.targets)))
        else:
            jevois.sendSerial("FN{}".format(min(len(self.targets), 9)))

    def processNoUSB(self, inframe):
        """
        Process the inframe without a USB connection to JeVois Inventor. This is run automatically by the camera.
        :param inframe:
        :return:
        """
        # We draw onto inimg for the purpose of saving images
        inimg = inframe.get()

        imgbgr = jevois.convertToCvBGR(inimg)
        inframe.done()

        # Get a list of quadrilateral convex hulls for all good objects:
        imgth, tapes = self.detect(imgbgr)
        self.draw_tapes(inimg, tapes)

        self.targets = self.define_targets(tapes)
        self.draw_targets(inimg, self.targets)

        if self.current_target[1] is None:
            self.choose_target(inimg)

        if self.current_target[1] is not None:
            self.track(inimg)

        if self.enabled:
            self.send_all_serial()

        if self.save_frame > 0:
            self.save_image(jevois.convertToCvBGR(inimg))

    def process(self, inframe, outframe):
        """
        Process the inframe with USB connection to JeVois Inventor. This is run automatically by the camera.
        :param inframe:
        :param outframe:
        :return:
        """
        inimg = inframe.get()
        outimg = outframe.get()
        outimg.require("output", 1280, 1024, jevois.V4L2_PIX_FMT_YUYV)

        self.draw_image(outimg, inimg)
        imgbgr = jevois.convertToCvBGR(inimg)
        inframe.done()

        # Get a list of quadrilateral convex hulls for all good objects:
        imgth, tapes = self.detect(imgbgr)
        self.draw_tapes(outimg, tapes)

        self.targets = self.define_targets(tapes)
        self.draw_targets(outimg, self.targets)

        if self.current_target[1] is None:
            self.choose_target(outimg)

        if self.current_target[1] is not None:
            self.track(outimg)

        if self.enabled:
            self.send_all_serial()

        if self.save_frame > 0:
            self.save_image(jevois.convertToCvBGR(outimg))

        outframe.send()

    def parseSerial(self, string : str):
        """
        Processes user commands
        :param string:
        :return:
        """
        # Start sending data
        if string[0] == 'S':
            return self.start_sending(string[1:])

        # Stop sending data
        if string == 'E':
            return self.end_sending()

        # Send data once
        if string == 's':
            self.send_all_serial()
            return ""

        # Save a frame
        if string.lower() == 'frame':
            self.current_time = date.strptime(string, "%Y%m%d%H%M%S")
            self.save_frame = 1
            return ""

        # Save several frames
        if string.lower().startswith('frames'):
            try:
                self.save_frame = int(string[6:])
            except ValueError:
                self.save_frame = 5
            return ""
        return ""

    def start_sending(self, string):
        """
        Start sending data over the serial
        :param string:
        :return:
        """
        self.send_all_serial()
        self.save_frame = 5
        self.enabled = True
        if string is not "":
            self.current_time = date.strptime(string, "%Y%m%d%H%M%S")
        return "Started Sending"

    def end_sending(self):
        """
        Stop sending data over the serial
        :return:
        """
        self.enabled = False
        return "Stopped Sending"

    def save_image(self, img):
        """
        Save img to a new .png file, tagged with a date
        :param img:
        :return:
        """
        file_name = 'data/ImageCapture {}.png'.format(self.current_time.strftime("%Y-%m-%d-%H-%M-%S"))
        jevois.LINFO("Saving current image to {}".format(file_name))
        cv2.imwrite(file_name, img)
        self.save_frame -= 1

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
        Finds the distance between between the camera's estimated translation and (0, y, 0)
        :param target_data:
        :return:
        """
        return math.sqrt(math.pow(target_data[2][0], 2) + math.pow(target_data[2][2], 2))

    def define_targets(self, tapes):
        """
        Organizes tape pieces into targets
        :param tapes:
        :return:
        """
        targets = []
        # Remove a leading right tape
        if len(tapes) >= 2 and tapes[0].parity is 'right' and tapes[1].parity is 'left':
            tapes.pop(0)
        # remove a trailing left tape
        if len(tapes) >= 2 and tapes[-1].parity is 'left' and tapes[-2].parity is 'right':
            tapes.pop(-1)

        # if len(tapes) % 2 == 0:
        #     while len(tapes) >= 2:
        #         targets.append(Target(tapes.pop(0), tapes.pop(0)))
        if len(tapes) >= 2:
            # Group up every possible combination of tapes, removing unlikely targets
            first_tape = tapes[0]
            i = 1
            while i < len(tapes) - 1:
                second_tape = tapes[i]
                target = Target(first_tape, second_tape)
                if (first_tape.parity is 'right' and second_tape.parity is 'left') is False and \
                        self.is_target_plausible(target):
                    targets.append(Target(first_tape, second_tape))
                first_tape = tapes[i]
                i += 1
            if i is len(tapes) - 1:
                second_tape = tapes[i]
                target = Target(first_tape, second_tape)
                if (first_tape.parity is 'right' and second_tape.parity is 'left') is False and \
                        self.is_target_plausible(target):
                    targets.append(Target(first_tape, second_tape))

        return targets

    def choose_target(self, outimg=None):
        """
        Chooses a target closest to the camera. If multiple targets are almost the same distance from the camera, return None
        :param outimg:
        :return:
        """
        targets = self.targets
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
                if self.is_target_plausible(closest_target) is False:
                    return
                closest_box = closest_target.bounding_rectangle
                self.current_target = np.array([[closest_target, rotation, translation], closest_box])
                jevois.LINFO("Target found")

    def track(self, outimg=None):
        """
        Updates tracking on the currently selected target
        :param outimg:
        :return:
        """
        success, target = self.find_target_closest_to_current()
        if success:
            self.target_lost_time = -1
            rotation, translation = self.estimate_pose(target)
            self.current_target[0] = (target, rotation, translation)
            self.current_target[1] = target.bounding_rectangle
            if outimg is not None:
                target.draw_trapezoid(outimg, 0x048f)
                rstring = "Rotation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(float(rotation[0]), float(
                    rotation[1]), float(rotation[2]))
                jevois.writeText(outimg, rstring, 3, 3, jevois.YUYV.White, jevois.Font.Font6x10)
                tstring = "Translation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(float(translation[0]),
                                                                                float(translation[1]),
                                                                                float(translation[2]))
                jevois.writeText(outimg, tstring, 3, 15, jevois.YUYV.White, jevois.Font.Font6x10)
        else:
            if self.target_lost_time != -1 and self.target_lost_timeout - date.now().second + self.target_lost_time > \
                    self.target_lost_timeout:
                self.target_lost_time = -1
            self.current_target[0] = None
            if self.target_lost_time is -1:
                self.target_lost_time = date.now().second
            if date.now().second - self.target_lost_time < self.target_lost_timeout:
                if outimg is not None:
                    jevois.writeText(outimg, "Target lost, waiting {} more seconds".format(self.target_lost_timeout -
                                                                                           date.now().second + self.target_lost_time),
                                     3, 3, jevois.YUYV.White,
                                     jevois.Font.Font6x10)
                jevois.LINFO("Target lost, waiting {} more seconds".format(self.target_lost_timeout -
                                                                           date.now().second + self.target_lost_time))
            else:
                self.current_target[1] = None
                self.target_lost_time = -1
                jevois.writeText(outimg, "Lost target has been timed out", 3, 3, jevois.YUYV.White,
                                 jevois.Font.Font6x10)
                jevois.LINFO("Lost target has been timed out")

    def find_target_closest_to_current(self):
        """
        Finds the target in targets closest to fitting inside of rectangle
        :return:
        """
        targets = self.targets
        if len(targets) is 0:
            return False, None
        targets.sort(key=lambda target: distance_2d(self.current_target[1], target.bounding_rectangle))
        most_likely_target = targets[0]
        if distance_2d(most_likely_target.bounding_rectangle, self.current_target[1]) > \
                most_likely_target.bounding_rectangle[3] * self.target_lost_factor:
            jevois.LINFO("Closest target is too far away")
            return False, None
        return True, targets[0]

    def is_target_plausible(self, target):
        rotation, translation = self.estimate_pose(target)
        # jevois.LINFO("{} {}".format(rotation, translation))
        return self.max_x_ang >= rotation[0] >= self.min_x_ang and self.max_z_ang >= rotation[2] >= self.min_z_ang \
               and translation[1] >= self.min_y and translation[2] >= 0


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
    def bounding_trapezoid(self):
        """
        Returns points corresponding to the smallest area quadrilateral surrounding a target. Parallel bases are left
        and right sides
        :return:
        """
        points = self.tape_corners
        top_slope = (points[5][1] - points[0][1]) / (points[5][0] - points[0][0])
        bottom_slope = (points[7][1] - points[2][1]) / (points[7][0] - points[2][0])
        tl = (points[3][0], top_slope * (points[3][0] - points[0][0]) + points[0][1])
        tr = (points[6][0], top_slope * (points[6][0] - points[5][0]) + points[5][1])
        br = (points[6][0], bottom_slope * (points[6][0] - points[7][0]) + points[7][1])
        bl = (points[3][0], bottom_slope * (points[3][0] - points[2][0]) + points[2][1])
        return np.array([tl, tr, br, bl])

    @property
    def tape_corners(self):
        points_left = self.left_tape.corners
        points_right = self.right_tape.corners
        return np.concatenate((points_left, points_right))

    @property
    def bounding_rectangle(self):
        """
        Returns the values corresponding to the bounding rectangle of the target
        :return: x - the x value of the top left corner, y - the y value of the top left corner, w - the width,
        h - the height
        """
        (x, y), tr, br, bl = self.bounding_trapezoid
        w = max(br[0], tr[0]) - x
        h = max(br[1], bl[1]) - y
        return x, y, w, h

    def draw_trapezoid(self, outimg, color=jevois.YUYV.LightPurple):
        """
        Draws the bounding_quadrilateral onto outimg
        :param outimg:
        :param color:
        :return:
        """
        if self.left_tape is not None and self.right_tape is not None:
            bbox = self.bounding_trapezoid
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
        if self.angle == -90 or self.angle == 0:
            return ''
        elif self.angle < -45:
            return 'left'
        elif self.angle > -45:
            return 'right'
        return ''

    @property
    def box_points(self):
        return cv2.boxPoints(cv2.minAreaRect(self.contour))

    @property
    def corners(self):
        """
        :return: box_points as top left, top right, bottom right, bottom left
        """
        return order_corners_clockwise(self)

    def draw(self, outimg):
        """
        Draws the minimum area rectangle of the tape onto outimg
        :param outimg:
        :return:
        """
        if self.contour.size > 0:
            if self.parity is 'left':
                color = jevois.YUYV.LightPurple
            elif self.parity is 'right':
                color = jevois.YUYV.DarkGreen
            else:
                color = jevois.YUYV.MedGrey
            point0 = self.box_points[-1]

            for point1 in self.box_points:
                jevois.drawLine(outimg, int(point0[0]), int(point0[1]),
                                int(point1[0]), int(point1[1]), 1, color)

                point0 = point1

import itertools as it
import math  # for cos, sin, etc
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
        # S: 0 for unsaturated (whitish discolored object) to 255 for fully saturated (solid color)
        # V: 0 for dark to 255 for maximally bright
        self.HSVmin = np.array([73, 110, 111], dtype=np.uint8)
        self.HSVmax = np.array([102, 255, 255], dtype=np.uint8)
        self.min_area = 200.0
        self.max_area = 1000.0
        self.min_perimeter = 10.0
        self.min_width = 5.0
        self.max_width = 1000.0
        self.min_height = 0.0
        self.max_height = 1000.0
        self.solidity = [0, 100]
        self.max_vertices = 1000000.0
        self.min_vertices = 4.0
        self.min_ratio = 0.5
        self.max_ratio = 2.0

        self.object_points = np.array(
            [[-5.936, 31.5, 0.0], [-4.0, 31, 0.0], [-5.375, 25.675, 0.0], [-7.316, 26.175, 0.0],  # Left points
             [4.0, 31, 0.0], [5.936, 31.5, 0.0], [7.316, 26.175, 0.0], [5.375, 25.675, 0.0]])  # Right points
        self.object_points = self.object_points.astype('float32')

        self.current_target = None

        self.decision_tolerance = 0.05

        # Instantiate a JeVois Timer to measure our processing framerate:
        self.timer = jevois.Timer("DeepSpacePoseFinder", 100, jevois.LOG_INFO)

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
            jevois.LINFO("Loaded camera calibration from {}".format(cpf))
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

        # Convert input image to HSV:
        imghsv = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2HSV)

        # Isolate pixels inside our desired HSV range:
        imgth = cv2.inRange(imghsv, self.HSVmin, self.HSVmax)

        # Apply morphological operations to cleanup the image noise:
        imgth = cv2.erode(imgth, self.erode_element)
        imgth = cv2.dilate(imgth, self.dilate_element)

        # Detect objects by finding contours:
        contours, hierarchy = cv2.findContours(imgth, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # Filter Contours by...
        output = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            # Width
            if w < self.min_width or w > self.max_width:
                continue
            # Height
            if h < self.min_height or h > self.max_height:
                continue
            area = cv2.contourArea(contour)
            # Area
            if area < self.min_area or area > self.max_area:
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
            ratio = float(w) / h
            # W / H ratio
            if ratio < self.min_ratio or ratio > self.max_ratio:
                continue
            output.append(TapePiece(hull))

        # Sort the tapes from left to right
        sorted_output = sorted(output, key=lambda tape: cv2.boundingRect(tape)[0])

        return imgth, sorted_output

    def draw_image(self, outimg, inimg):

        if outimg.valid():
            jevois.paste(inimg, outimg, 0, 0)

    def draw_tapes(self, outimg, tapes):
        # Display any results requested by the users:

        if outimg.valid():
            for tape in tapes:
                tape.draw(outimg)

    def draw_targets(self, outimg, targets):
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
            if target.left_tape is not None and target.right_tape is not None:
                image_points_left = np.array(self.order_corners_clockwise(target.left_rect))
                image_points_left = image_points_left.astype('float32')

                image_points_right = np.array(self.order_corners_clockwise(target.right_rect))
                image_points_right = image_points_right.astype('float32')

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
        image_points_left = np.array(self.order_corners_clockwise(target.left_rect))
        image_points_left = image_points_left.astype('float32')

        image_points_right = np.array(self.order_corners_clockwise(target.right_rect))
        image_points_right = image_points_right.astype('float32')

        image_points = np.concatenate((image_points_left, image_points_right))
        ret, rotation, translation = cv2.solvePnP(self.object_points, image_points, self.cam_matrix,
                                                  self.dist_coeffs)
        return rotation, translation

    @staticmethod
    def send_all_serial(pose, targets):
        """
        Sends a message over the SerialPort to the RoboRIO
        :param pose:
        :param targets:
        :return:
        """
        if pose is not None:

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

        imgbgr = jevois.convertToCvBGR(inimg)
        inframe.done()

        # Start measuring image processing time:
        #self.timer.start()

        # Convert input image to BGR24:

        # Get pre-allocated but blank output image which we will send over USB:

        # Get a list of quadrilateral convex hulls for all good objects:
        # tapes = self.detect(imgbgr)

        # Load camera calibration if needed:
        # if not hasattr(self, 'camMatrix'): self.load_camera_calibration(res_w, res_h)
        #
        # contour_tracker = FindTargets()
        #
        # targets = contour_tracker.find_full_contours(contours, (res_w, res_h))
        # if len(targets) > 0:
        #     target_data = []
        #     for target in targets:
        #         # Map to 6D (inverse perspective):
        #         try:
        #             rvecs, tvecs = self.estimate_pose(target)
        #             target_data.append((target, rvecs, tvecs))
        #         except SystemError:
        #             jevois.LINFO("Oops that contour is a no bueno")
        #
        #     target_data.sort(key = self.distance_from_origin)
        #
        #     for target, rvecs, tvecs in target_data:
        #         # Draw all detections in 3D:
        #         self.draw_lines(outimg, target, rvecs, tvecs)
        #
        #     if len(target_data) != 1 and self.percent_difference(self.distance_from_origin(target_data[0]),
        #                                self.distance_from_origin(target_data[1])) <= self.decision_tolerance:
        #         jevois.writeText(outimg, "cannot decide which target to go to", 3, 3, jevois.YUYV.White,
        #                          jevois.Font.Font6x10)
        #         # Send all serial messages:
        #         jevois.sendSerial("FN{}".format(len(targets)))
        #     else:
        #         closest_target, rvecs, tvecs = target_data[0]
        #         self.draw_lines(outimg, closest_target, rvecs, tvecs, True)
        #
        #         rstring = "Rotation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(math.degrees(rvecs[0]), math.degrees(rvecs[1]),
        #                                                                      math.degrees(rvecs[2]))
        #         jevois.writeText(outimg, rstring, 3, 3, jevois.YUYV.White, jevois.Font.Font6x10)
        #         tstring = "Translation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(float(tvecs[0]), float(tvecs[1]),
        #                                                                         float(tvecs[2]))
        #         jevois.writeText(outimg, tstring, 3, 15, jevois.YUYV.White, jevois.Font.Font6x10)
        #
        #         Pose.x = tvecs[2]
        #         Pose.y = tvecs[0]
        #         Pose.z = tvecs[1]
        #         Pose.angle = -rvecs[2]
        #         self.send_all_serial(Pose, targets)
        #
        #     # Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
        #     fps = self.timer.stop()
        #     jevois.writeText(outimg, fps, 3, res_h - 10, jevois.YUYV.White, jevois.Font.Font6x10)
        #
        # else:
        #     jevois.writeText(outimg, 'no target detected', 3, 3, jevois.YUYV.White, jevois.Font.Font6x10)
        #     jevois.sendSerial("FN{}".format(0))

        # We are done with the output, ready to send it to host over USB:

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

        # Start measuring image processing time:
        self.timer.start()

        # Convert input image to BGR24:

        # Get pre-allocated but blank output image which we will send over USB:

        # Get a list of quadrilateral convex hulls for all good objects:
        imgth, tapes = self.detect(imgbgr)
        self.draw_tapes(outimg, tapes)

        targets = self.define_targets(tapes)
        self.draw_targets(outimg, targets)

        if self.current_target is None:
            self.choose_target(imgth, targets, outimg)
        else:
            self.track(imgth, targets)

        # Load camera calibration if needed:
        # if not hasattr(self, 'camMatrix'): self.load_camera_calibration(res_w, res_h)
        #
        # contour_tracker = FindTargets()
        #
        # targets = contour_tracker.find_full_contours(contours, (res_w, res_h))
        # if len(targets) > 0:
        #     target_data = []
        #     for target in targets:
        #         # Map to 6D (inverse perspective):
        #         try:
        #             rvecs, tvecs = self.estimate_pose(target)
        #             target_data.append((target, rvecs, tvecs))
        #         except SystemError:
        #             jevois.LINFO("Oops that contour is a no bueno")
        #
        #     target_data.sort(key = self.distance_from_origin)
        #
        #     for target, rvecs, tvecs in target_data:
        #         # Draw all detections in 3D:
        #         self.draw_lines(outimg, target, rvecs, tvecs)
        #
        #     if len(target_data) != 1 and self.percent_difference(self.distance_from_origin(target_data[0]),
        #                                self.distance_from_origin(target_data[1])) <= self.decision_tolerance:
        #         jevois.writeText(outimg, "cannot decide which target to go to", 3, 3, jevois.YUYV.White,
        #                          jevois.Font.Font6x10)
        #         # Send all serial messages:
        #         jevois.sendSerial("FN{}".format(len(targets)))
        #     else:
        #         closest_target, rvecs, tvecs = target_data[0]
        #         self.draw_lines(outimg, closest_target, rvecs, tvecs, True)
        #
        #         rstring = "Rotation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(math.degrees(rvecs[0]), math.degrees(rvecs[1]),
        #                                                                      math.degrees(rvecs[2]))
        #         jevois.writeText(outimg, rstring, 3, 3, jevois.YUYV.White, jevois.Font.Font6x10)
        #         tstring = "Translation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(float(tvecs[0]), float(tvecs[1]),
        #                                                                         float(tvecs[2]))
        #         jevois.writeText(outimg, tstring, 3, 15, jevois.YUYV.White, jevois.Font.Font6x10)
        #
        #         Pose.x = tvecs[2]
        #         Pose.y = tvecs[0]
        #         Pose.z = tvecs[1]
        #         Pose.angle = -rvecs[2]
        #         self.send_all_serial(Pose, targets)
        #
        #     # Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
        #     fps = self.timer.stop()
        #     jevois.writeText(outimg, fps, 3, res_h - 10, jevois.YUYV.White, jevois.Font.Font6x10)
        #
        # else:
        #     jevois.writeText(outimg, 'no target detected', 3, 3, jevois.YUYV.White, jevois.Font.Font6x10)
        #     jevois.sendSerial("FN{}".format(0))

        # We are done with the output, ready to send it to host over USB:
        outframe.send()

    def order_corners_clockwise(self, contour):
        """
        Sorts the corners of a contour clockwise from the top left point
        :param contour: the contour to sort the corners of
        :return: the sorted corners
        """
        # get corners of contour
        corners = cv2.boxPoints(contour)
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
        return np.array([tl, tr, br, bl], dtype="float32")

    def percent_difference(self, value1, value2):
        """
        Finds the percent difference between two values, between 0.0 and 1.0
        :param value1:
        :param value2:
        :return:
        """
        return math.fabs(value1 - value2) / (value1 + value2)

    def distance_from_origin(self, target_data):
        """
        :param target_data:
        :return: the distance between point and (0, y, 0)
        """
        return math.sqrt(math.pow(target_data[2][0], 2) + math.pow(target_data[2][2], 2))

    def define_targets(self, tapes):
        targets = []
        if tapes[0].parity == 'right':
            targets.append(Target(None, tapes.pop(0)))
        while len(tapes) >= 2:
            left_tape = tapes.pop(0)
            right_tape = tapes.pop(0)
            targets.append(Target(left_tape, right_tape))

        # If there is a Tape left over, it is the left tape of a Target
        if len(tapes) > 0:
            targets.append(Target(tapes.pop(0), None))
        return targets

    def choose_target(self, inimg, targets, outimg=None):
        """
        Chooses a target closest to the camera. If multiple targets are almost the same distance from the camera, return None
        :param inimg:
        :param targets:
        :param outimg:
        :return:
        """
        rotations, translations = self.estimate_poses(targets)
        target_data = []
        for i, target in enumerate(targets):
            target_data.append((target, rotations[i], translations[i]))
        target_data.sort(key=self.distance_from_origin)

        if len(target_data) is 0:
            jevois.LINFO("No target found")
        elif len(target_data) != 1 and self.percent_difference(self.distance_from_origin(target_data[0]),
                                                             self.distance_from_origin(target_data[1])) <= \
                self.decision_tolerance:
            jevois.LINFO("Cannot decide which target to choose")
            if outimg is not None:
                jevois.writeText(outimg, "Cannot decide which target to choose", 3, 3, jevois.YUYV.White,
                                 jevois.Font.Font6x10)
        else:
            closest_target, rotation, translation = target_data[0]
            tracker = cv2.TrackerCSRT_create()
            tracker.init(inimg, closest_target.bounding_rectangle)
            self.current_target = (target_data, tracker)
            jevois.LINFO("Target found")

    def track(self, inimg, targets, outimg=None):
        """
        Updates tracking on the currently selected target
        :param inimg:
        :param targets:
        :param outimg:
        :return:
        """
        tracker = self.current_target[1]
        success, bounding_rect = tracker.update(inimg)
        if success:
            target = self.find_target_in_rectangle(targets, bounding_rect)
            rotation, translation = self.estimate_pose(target)
            self.current_target[0] = (target, rotation, translation)
            if outimg is not None:
                target.draw_rectangle(outimg)
                rstring = "Rotation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(math.degrees(rotation[0]), math.degrees(
                    rotation[1]), math.degrees(rotation[2]))
                jevois.writeText(outimg, rstring, 3, 3, jevois.YUYV.White, jevois.Font.Font6x10)
                tstring = "Translation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(float(translation[0]),
                                                                                float(translation[1]),
                                                                                float(translation[2]))
                jevois.writeText(outimg, tstring, 3, 15, jevois.YUYV.White, jevois.Font.Font6x10)
        else:
            self.current_target = None
            if outimg is not None:
                jevois.writeText(outimg, "Target lost", 3, 3, jevois.YUYV.White,
                                 jevois.Font.Font6x10)
            jevois.LINFO("Target lost")

    def find_target_in_rectangle(self, targets, rectangle):
        """
        Finds the target in targets closest to fitting inside of rectangle
        :param targets:
        :param rectangle:
        :return:
        """
        targets.sort(key=lambda target: distance_2d(rectangle, target.bounding_rectangle))
        return targets[0]

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
        for point in cv2.boxPoints(self.left_tape):
            bl[0] = min(float(bl[0]), float(point[0]))
            bl[1] = max(float(bl[1]), float(point[1]))
            tl[0] = min(float(tl[0]), float(point[0]))
            tl[1] = min(float(tl[1]), float(point[1]))

        br = np.array([-math.inf, -math.inf])
        tr = np.array([-math.inf, math.inf])
        for point in cv2.boxPoints(self.right_tape):
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

    def draw_quadrilateral(self, outimg):
        if self.left_tape is not None and self.right_tape is not None:
            bbox = self.bounding_quadrilateral
            point0 = bbox[-1]
            for point1 in bbox:
                jevois.drawLine(outimg, int(point0[0]), int(point0[1]), int(point1[0]), int(point1[1]), 1,
                                jevois.YUYV.LightPurple)
                point0 = point1

    def draw_rectangle(self, outimg):
        if self.left_tape is not None and self.right_tape is not None:
            bbox = self.bounding_rectangle
            point0 = bbox[-1]
            for point1 in bbox:
                jevois.drawLine(outimg, int(point0[0]), int(point0[1]), int(point1[0]), int(point1[1]), 1,
                                jevois.YUYV.LightPurple)
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

        if self.angle > -45:
            return 'right'

        else:
            return 'left'

    def draw(self, outimg):
        if self.contour.size > 0:
            point0 = self.contour[-1]

            for point1 in self.contour:
                jevois.drawLine(outimg, int(point0[0, 0]), int(point0[0, 1]),
                                int(point1[0, 0]), int(point1[0, 1]), 1, jevois.YUYV.MedGreen)

                point0 = point1
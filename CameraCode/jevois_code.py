import libjevois as jevois
import cv2
import numpy as np
import math  # for cos, sin, etc
import itertools as it

colors = [
    (0, 0, 255),
    (0, 255, 0),
]


def pairwise(iterable):
    a = iter(iterable)
    return it.zip_longest(a, a, fillvalue=None)

class Pose:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle = 0

class DeepSpacePoseFinder:
    # ###################################################################################################
    ## Constructor
    def __init__(self):
        # HSV color range to use:
        #
        # H: 0=red/do not use because of wraparound, 30=yellow, 45=light green, 60=green, 75=green cyan, 90=cyan,
        #      105=light blue, 120=blue, 135=purple, 150=pink
        # S: 0 for unsaturated (whitish discolored object) to 255 for fully saturated (solid color)
        # V: 0 for dark to 255 for maximally bright
        self.HSVmin = np.array([73, 110, 111], dtype=np.uint8)
        self.HSVmax = np.array([102, 255, 255], dtype=np.uint8)
        self.min_area = 15.0
        self.min_perimeter = 10.0
        self.min_width = 0.0
        self.max_width = 1000.0
        self.min_height = 0.0
        self.max_height = 1000.0
        self.solidity = [0, 100]
        self.max_vertices = 1000000.0
        self.min_vertices = 0.0
        self.min_ratio = 0.0
        self.max_ratio = 1000.0

        self.decision_tolerance = 0.1

        # Measure your U-shaped object (in meters) and set its size here:
        self.owm = 0.280  # width in meters
        self.ohm = 0.175  # height in meters

        # Other processing parameters:
        self.epsilon = 0.015  # Shape smoothing factor (higher for smoother)
        self.hullarea = (20 * 20, 300 * 300)  # Range of object area (in pixels) to track
        self.hullfill = 50  # Max fill ratio of the convex hull (percent)
        self.ethresh = 900  # Shape error threshold (lower is stricter for exact shape)
        self.margin = 5  # Margin from from frame borders (pixels)

        # Instantiate a JeVois Timer to measure our processing framerate:
        self.timer = jevois.Timer("FirstPython", 100, jevois.LOG_INFO)

        # CAUTION: The constructor is a time-critical code section. Taking too long here could upset USB timings and/or
        # video capture software running on the host computer. Only init the strict minimum here, and do not use OpenCV,
        # read files, etc

    # ###################################################################################################
    ## Load camera calibration from JeVois share directory
    def load_camera_calibration(self, w, h):
        cpf = "/jevois/share/camera/calibration{}x{}.yaml".format(w, h)
        fs = cv2.FileStorage(cpf, cv2.FILE_STORAGE_READ)
        if fs.isOpened():
            self.cam_matrix = fs.getNode("camera_matrix").mat()
            self.dist_coeffs = fs.getNode("distortion_coefficients").mat()
            jevois.LINFO("Loaded camera calibration from {}".format(cpf))
        else:
            jevois.LFATAL("Failed to read camera parameters from file [{}]".format(cpf))

    # ###################################################################################################
    ## Detect objects within our HSV range
    def detect(self, imgbgr, outimg=None):
        res_h, res_w, chans = imgbgr.shape

        # Convert input image to HSV:
        imghsv = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2HSV)

        # Isolate pixels inside our desired HSV range:
        imgth = cv2.inRange(imghsv, self.HSVmin, self.HSVmax)
        str = "H={}-{} S={}-{} V={}-{} ".format(self.HSVmin[0], self.HSVmax[0], self.HSVmin[1],
                                                self.HSVmax[1], self.HSVmin[2], self.HSVmax[2])

        # Create structuring elements for morpho maths:
        if not hasattr(self, 'erodeElement'):
            self.erode_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            self.dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))

        # Apply morphological operations to cleanup the image noise:
        imgth = cv2.erode(imgth, self.erode_element)
        imgth = cv2.dilate(imgth, self.dilate_element)

        # Detect objects by finding contours:
        contours, hierarchy = cv2.findContours(imgth, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        str += "N={} ".format(len(contours))

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
            ratio = float(w) / h
            # W / H ratio
            if ratio < self.min_ratio or ratio > self.max_ratio:
                continue
            output.append(contour)

        # Display any results requested by the users:
        if outimg is not None and outimg.valid():
            jevois.pasteGreyToYUYV(imgth, outimg, 0, 0)
            jevois.writeText(outimg, str, 3, res_h + 1, jevois.YUYV.White, jevois.Font.Font6x10)
            for contour in output:
                box = cv2.boxPoints(cv2.minAreaRect(contour))
                point0 = box[-1]
                for point1 in box:
                    jevois.drawLine(outimg, int(point0[0]), int(point0[1]), int(point1[0]), int(point1[1]), 1,
                                    jevois.YUYV.LightPurple)
                    point0 = point1

        return output

    # ###################################################################################################
    ## Estimate 6D pose of each of the quadrilateral objects in hlist:
    def estimate_pose(self, target):
        object_points = np.array(
            [[-5.936, 31.5, 0.0], [-4.0, 31, 0.0], [-5.375, 25.675, 0.0], [-7.316, 26.175, 0.0],  # Left points
             [4.0, 31, 0.0], [5.936, 31.5, 0.0], [7.316, 26.175, 0.0], [5.375, 25.675, 0.0]])  # Right points
        object_points = object_points.astype('float32')

        image_points_left = np.array(self.order_corners_clockwise(target.left_rect))
        image_points_left = image_points_left.astype('float32')

        image_points_right = np.array(self.order_corners_clockwise(target.right_rect))
        image_points_right = image_points_right.astype('float32')

        image_points = np.concatenate((image_points_left, image_points_right))

        ret, rotation, translation = cv2.solvePnP(object_points, image_points, self.cam_matrix, self.dist_coeffs)
        return rotation, translation

    # ###################################################################################################
    ## Send serial messages, one per object
    @staticmethod
    def send_all_serial(pose, targets):


        if pose is not None:

            x_key = 'x' if pose.x < 0 else 'X'
            x = min(int(round(abs(pose.x) * 2.54)), 999)


            y_key = 'z' if pose.z < 0 else 'Y'
            y = min(int(round(abs(pose.y) * 2.54)), 999)


            z_key = 'z' if pose.z < 0 else 'Z'
            z = min(int(round(abs(pose.z) * 2.54)), 99)


            angle_key = 'a' if pose.angle < 0 else 'A'

            angle = min(int(round(math.degrees(pose.angle))), 999)

            jevois.sendSerial("{0:s}{1:3d}{2:s}{3:3d}{4:s}{5:2d}{6:s}{7:3d}N{8:1d}".format(x_key, x, y_key, y, z_key, z, angle_key, angle, targets)) # pose

        else:
            jevois.sendSerial("FN{}".format(min(len(targets)), 9))

    # ###################################################################################################
    ## Draw all detected objects in 3D
    # def drawDetections(self, outimg, hlist, rvecs=None, tvecs=None):
    #     # Show trihedron and parallelepiped centered on object:
    #     hw = self.owm * 0.5
    #     hh = self.ohm * 0.5
    #     dd = -max(hw, hh)
    #     i = 0
    #     empty = np.array([(0.0), (0.0), (0.0)])
    #
    #     for obj in hlist:
    #         # skip those for which solvePnP failed:
    #         if np.array_equal(rvecs[i], empty):
    #             i += 1
    #             continue
    #
    #         # Project axis points:
    #         axisPoints = np.array([(0.0, 0.0, 0.0), (hw, 0.0, 0.0), (0.0, hh, 0.0), (0.0, 0.0, dd)])
    #         imagePoints, jac = cv2.projectPoints(axisPoints, rvecs[i], tvecs[i], self.cam_matrix, self.dist_coeffs)
    #
    #         # Draw axis lines:
    #         jevois.drawLine(outimg, int(imagePoints[0][0, 0] + 0.5), int(imagePoints[0][0, 1] + 0.5),
    #                         int(imagePoints[1][0, 0] + 0.5), int(imagePoints[1][0, 1] + 0.5),
    #                         2, jevois.YUYV.MedPurple)
    #         jevois.drawLine(outimg, int(imagePoints[0][0, 0] + 0.5), int(imagePoints[0][0, 1] + 0.5),
    #                         int(imagePoints[2][0, 0] + 0.5), int(imagePoints[2][0, 1] + 0.5),
    #                         2, jevois.YUYV.MedGreen)
    #         jevois.drawLine(outimg, int(imagePoints[0][0, 0] + 0.5), int(imagePoints[0][0, 1] + 0.5),
    #                         int(imagePoints[3][0, 0] + 0.5), int(imagePoints[3][0, 1] + 0.5),
    #                         2, jevois.YUYV.MedGrey)
    #
    #         # Also draw a parallelepiped:
    #         cubePoints = np.array([(-hw, -hh, 0.0), (hw, -hh, 0.0), (hw, hh, 0.0), (-hw, hh, 0.0),
    #                                (-hw, -hh, dd), (hw, -hh, dd), (hw, hh, dd), (-hw, hh, dd)])
    #         cu, jac2 = cv2.projectPoints(cubePoints, rvecs[i], tvecs[i], self.cam_matrix, self.dist_coeffs)
    #
    #         # Round all the coordinates and cast to int for drawing:
    #         cu = np.rint(cu)
    #
    #         # Draw parallelepiped lines:
    #         jevois.drawLine(outimg, int(cu[0][0, 0]), int(cu[0][0, 1]), int(cu[1][0, 0]), int(cu[1][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #         jevois.drawLine(outimg, int(cu[1][0, 0]), int(cu[1][0, 1]), int(cu[2][0, 0]), int(cu[2][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #         jevois.drawLine(outimg, int(cu[2][0, 0]), int(cu[2][0, 1]), int(cu[3][0, 0]), int(cu[3][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #         jevois.drawLine(outimg, int(cu[3][0, 0]), int(cu[3][0, 1]), int(cu[0][0, 0]), int(cu[0][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #         jevois.drawLine(outimg, int(cu[4][0, 0]), int(cu[4][0, 1]), int(cu[5][0, 0]), int(cu[5][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #         jevois.drawLine(outimg, int(cu[5][0, 0]), int(cu[5][0, 1]), int(cu[6][0, 0]), int(cu[6][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #         jevois.drawLine(outimg, int(cu[6][0, 0]), int(cu[6][0, 1]), int(cu[7][0, 0]), int(cu[7][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #         jevois.drawLine(outimg, int(cu[7][0, 0]), int(cu[7][0, 1]), int(cu[4][0, 0]), int(cu[4][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #         jevois.drawLine(outimg, int(cu[0][0, 0]), int(cu[0][0, 1]), int(cu[4][0, 0]), int(cu[4][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #         jevois.drawLine(outimg, int(cu[1][0, 0]), int(cu[1][0, 1]), int(cu[5][0, 0]), int(cu[5][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #         jevois.drawLine(outimg, int(cu[2][0, 0]), int(cu[2][0, 1]), int(cu[6][0, 0]), int(cu[6][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #         jevois.drawLine(outimg, int(cu[3][0, 0]), int(cu[3][0, 1]), int(cu[7][0, 0]), int(cu[7][0, 1]),
    #                         1, jevois.YUYV.LightGreen)
    #
    #         i += 1

    def draw_lines(self, outimg, target, rvecs=None, tvecs=None, is_closest=False):
        # Draw rotated axis lines
        axis_points = np.array([[0.0, 28.875, 0.0],
                                [0.0, 28.875, 4.0]])
        projected_axis_points, jac = cv2.projectPoints(axis_points, rvecs, tvecs, self.cam_matrix, self.dist_coeffs)
        point0 = projected_axis_points[0]
        point1 = projected_axis_points[1]
        jevois.LINFO("{} {}".format(point0[0, 0], point0[0, 1]))
        if is_closest:
            jevois.drawLine(outimg, int(point0[0, 0]), int(point0[0, 1]),
                            int(point1[0, 0]), int(point1[0, 1]), 1, jevois.YUYV.MedPurple)
        else:
            jevois.drawLine(outimg, int(point0[0, 0]), int(point0[0, 1]),
                            int(point1[0, 0]), int(point1[0, 1]), 1, jevois.YUYV.MedGreen)

        box = target.bounding_quadrilateral
        point0 = box[-1]
        for point1 in box:
            if is_closest:
                jevois.drawLine(outimg, int(point0[0]), int(point0[1]), int(point1[0]), int(point1[1]), 1,
                                jevois.YUYV.LightPurple)
            else:
                jevois.drawLine(outimg, int(point0[0]), int(point0[1]), int(point1[0]), int(point1[1]), 1,
                                jevois.YUYV.LightGreen)
            point0 = point1

    # ###################################################################################################
    ## Process function with no USB output
    def processNoUSB(self, inframe):
        # Get the next camera image (may block until it is captured). To avoid wasting much time assembling a composite
        # output image with multiple panels by concatenating numpy arrays, in this module we use raw YUYV images and
        # fast paste and draw operations provided by JeVois on those images:
        inimg = inframe.get()

        # Start measuring image processing time:
        self.timer.start()

        # Convert input image to BGR24:
        imgbgr = jevois.convertToCvBGR(inimg)
        res_h, res_w, chans = imgbgr.shape

        # Let camera know we are done using the input image:
        inframe.done()

        # Get a list of quadrilateral convex hulls for all good objects:
        contours = self.detect(imgbgr)

        # Load camera calibration if needed:
        if not hasattr(self, 'camMatrix'): self.load_camera_calibration(res_w, res_h)

        contour_tracker = FindTargets()
        targets = contour_tracker.find_full_contours(contours, (res_w, res_h))
        if targets is not None:
            target_data = []
            for target in targets:
                # Map to 6D (inverse perspective):
                rvecs, tvecs = self.estimate_pose(target)
                target_data.append((target, rvecs, tvecs))

            target_data.sort(key=self.distance_from_origin)


            if self.percent_difference(self.distance_from_origin(target_data[0]),
                                       self.distance_from_origin(target_data[1])) <= self.decision_tolerance:
                jevois.sendSerial("FN{}".format(len(targets)))

            else:
                closest_target, rvecs, tvecs = target_data[0]

                # Send all serial messages:
                self.send_all_serial(tvecs[0], tvecs[2], tvecs[1], rvecs[1], len(targets))

        else:
            jevois.sendSerial("FN{}".format(0))

            # Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
            fps = self.timer.stop()

        self.timer.stop()

    # ###################################################################################################
    ## Process function with USB output
    def process(self, inframe, outframe):

        # Get the next camera image (may block until it is captured). To avoid wasting much time assembling a composite
        # output image with multiple panels by concatenating numpy arrays, in this module we use raw YUYV images and
        # fast paste and draw operations provided by JeVois on those images:
        inimg = inframe.get()

        # Start measuring image processing time:
        self.timer.start()

        # Convert input image to BGR24:
        imgbgr = jevois.convertToCvBGR(inimg)
        res_h, res_w, chans = imgbgr.shape

        # Get pre-allocated but blank output image which we will send over USB:
        outimg = outframe.get()
        outimg.require("output", res_w, res_h, jevois.V4L2_PIX_FMT_YUYV)  # WHAT? *2 + 12?
        jevois.paste(inimg, outimg, 0, 0)
        jevois.drawFilledRect(outimg, 0, res_h, outimg.width, outimg.height - res_h, jevois.YUYV.Black)

        # Let camera know we are done using the input image:
        inframe.done()

        # Get a list of quadrilateral convex hulls for all good objects:
        contours = self.detect(imgbgr, outimg)

        # Load camera calibration if needed:
        if not hasattr(self, 'camMatrix'): self.load_camera_calibration(res_w, res_h)

        contour_tracker = FindTargets()

        targets = contour_tracker.find_full_contours(contours, (res_w, res_h))
        if targets is not None:
            target_data = []
            for target in targets:
                # Map to 6D (inverse perspective):
                rvecs, tvecs = self.estimate_pose(target)
                target_data.append((target, rvecs, tvecs))

            target_data.sort(key = self.distance_from_origin)

            for target, rvecs, tvecs in target_data:
                # Draw all detections in 3D:
                self.draw_lines(outimg, target, rvecs, tvecs)

            if self.percent_difference(self.distance_from_origin(target_data[0]),
                                       self.distance_from_origin(target_data[1])) <= self.decision_tolerance:
                jevois.writeText(outimg, "cannot decide which target to go to", 3, 3, jevois.YUYV.White,
                                 jevois.Font.Font6x10)
                # Send all serial messages:
            else:
                closest_target, rvecs, tvecs = target_data[0]
                self.draw_lines(outimg, closest_target, rvecs, tvecs, True)

                rstring = "Rotation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(math.degrees(rvecs[0]), math.degrees(rvecs[1]),
                                                                             math.degrees(rvecs[2]))
                jevois.writeText(outimg, rstring, 3, 3, jevois.YUYV.White, jevois.Font.Font6x10)
                tstring = "Translation = ({0:6.1f}, {1:6.1f}, {2:6.1f})".format(float(tvecs[0]), float(tvecs[1]),
                                                                                float(tvecs[2]))
                jevois.writeText(outimg, tstring, 3, 15, jevois.YUYV.White, jevois.Font.Font6x10)

                Pose.x = tvecs[2], Pose.y = tvecs[0], Pose.z = tvecs[1], -Pose.angle = rvecs[1]

            # Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
            fps = self.timer.stop()
            jevois.writeText(outimg, fps, 3, res_h - 10, jevois.YUYV.White, jevois.Font.Font6x10)

        else:
            jevois.writeText(outimg, 'no contour detected', 3, 3, jevois.YUYV.White, jevois.Font.Font6x10)
            jevois.sendSerial("FN{}".format(0))

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
        if self.distance_2d(tl, right_most[0]) > self.distance_2d(tl, right_most[1]):
            br, tr = right_most
        else:
            tr, br = right_most

        # return the coordinates in top-left, top-right, bottom-right, and bottom-left order
        return np.array([tl, tr, br, bl], dtype="float32")

    def distance_2d(self, point0, point1):
        """
        :param point0:
        :param point1:
        :return: the distance between point0 and point1
        """
        return math.sqrt(math.pow(point0[0] - point1[0], 2) + math.pow(point0[1] - point1[1], 2))

    def percent_difference(self, value1, value2):
        return math.fabs(value1 - value2) / (value1 + value2)

    def distance_from_origin(self, target_data):
        """
        :param point:
        :return: the distance between point and (0, y, 0)
        """
        return math.sqrt(math.pow(target_data[2][0], 2) + math.pow(target_data[2][2], 2))


class Target:

    def __init__(self, left_rect, right_rect):
        self.left_rect = left_rect
        self.right_rect = right_rect

    @property
    def average_center(self):
        return Target.average_x, Target.average_y

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

    @property
    def bounding_quadrilateral(self):
        bl = np.array([math.inf, -math.inf])
        tl = np.array([math.inf, math.inf])
        for point in cv2.boxPoints(self.left_rect):
            bl[0] = min(float(bl[0]), float(point[0]))
            bl[1] = max(float(bl[1]), float(point[1]))
            tl[0] = min(float(tl[0]), float(point[0]))
            tl[1] = min(float(tl[1]), float(point[1]))

        br = np.array([-math.inf, -math.inf])
        tr = np.array([-math.inf, math.inf])
        for point in cv2.boxPoints(self.right_rect):
            br[0] = max(float(br[0]), float(point[0]))
            br[1] = max(float(br[1]), float(point[1]))
            tr[0] = max(float(tr[0]), float(point[0]))
            tr[1] = min(float(tr[1]), float(point[1]))
        return (tl[0], tl[1]), (tr[0], tr[1]), (br[0], br[1]), (bl[0], bl[1])


class FindTargets:

    def __init__(self):
        pass

    def __get_min_area_rects(self, contours):
        contour_rects = []

        for contour in contours:
            rect = cv2.minAreaRect(contour)
            contour_rects.append(rect)

        return contour_rects

    def find_full_contours(self, contours, frame_size):

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

        # if len(remainder) > 0:
        #     remainder = min(remainder, key=lambda target: math.fabs(target.average_x - frame_size[1] / 2))
        # else:
        #     remainder = None

        return remainder


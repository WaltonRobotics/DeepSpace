import libjevois as jevois
import cv2
import numpy as np
import math # for cos, sin, etc
import itertools as it

colors = [
    (0, 0, 255),
    (0, 255, 0),
]

def pairwise(iterable):
    a = iter(iterable)
    return it.zip_longest(a, a, fillvalue=None)

class FirstPython:
    # ###################################################################################################
    ## Constructor
    def __init__(self):
        # HSV color range to use:
        #
        # H: 0=red/do not use because of wraparound, 30=yellow, 45=light green, 60=green, 75=green cyan, 90=cyan,
        #      105=light blue, 120=blue, 135=purple, 150=pink
        # S: 0 for unsaturated (whitish discolored object) to 255 for fully saturated (solid color)
        # V: 0 for dark to 255 for maximally bright
        self.HSVmin = np.array([ 80,  50, 180], dtype=np.uint8)
        self.HSVmax = np.array([ 90, 255, 255], dtype=np.uint8)
        self.min_area = 0.0
        self.min_perimeter = 0.0
        self.min_width = 0.0
        self.max_width = 1000.0
        self.min_height = 0.0
        self.max_height = 1000.0
        self.solidity = [0, 100]
        self.max_vertices = 1000000.0
        self.min_vertices = 0.0
        self.min_ratio = 0.0
        self.max_ratio = 1000.0

        # Measure your U-shaped object (in meters) and set its size here:
        self.owm = 0.280 # width in meters
        self.ohm = 0.175 # height in meters

        # Other processing parameters:
        self.epsilon = 0.015               # Shape smoothing factor (higher for smoother)
        self.hullarea = ( 20*20, 300*300 ) # Range of object area (in pixels) to track
        self.hullfill = 50                 # Max fill ratio of the convex hull (percent)
        self.ethresh = 900                 # Shape error threshold (lower is stricter for exact shape)
        self.margin = 5                    # Margin from from frame borders (pixels)
    
        # Instantiate a JeVois Timer to measure our processing framerate:
        self.timer = jevois.Timer("FirstPython", 100, jevois.LOG_INFO)

        # CAUTION: The constructor is a time-critical code section. Taking too long here could upset USB timings and/or
        # video capture software running on the host computer. Only init the strict minimum here, and do not use OpenCV,
        # read files, etc
        
    # ###################################################################################################
    ## Load camera calibration from JeVois share directory
    def loadCameraCalibration(self, w, h):
        cpf = "/jevois/share/camera/calibration{}x{}.yaml".format(w, h)
        fs = cv2.FileStorage(cpf, cv2.FILE_STORAGE_READ)
        if (fs.isOpened()):
            self.camMatrix = fs.getNode("camera_matrix").mat()
            self.distCoeffs = fs.getNode("distortion_coefficients").mat()
            jevois.LINFO("Loaded camera calibration from {}".format(cpf))
        else:
            jevois.LFATAL("Failed to read camera parameters from file [{}]".format(cpf))

    # ###################################################################################################
    ## Detect objects within our HSV range
    def detect(self, imgbgr, outimg = None):
        maxn = 5 # max number of objects we will consider
        res_h, res_w, chans = imgbgr.shape

        # Convert input image to HSV:
        imghsv = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2HSV)

        # Isolate pixels inside our desired HSV range:
        imgth = cv2.inRange(imghsv, self.HSVmin, self.HSVmax)
        str = "H={}-{} S={}-{} V={}-{} ".format(self.HSVmin[0], self.HSVmax[0], self.HSVmin[1],
                                                self.HSVmax[1], self.HSVmin[2], self.HSVmax[2])

        # Create structuring elements for morpho maths:
        if not hasattr(self, 'erodeElement'):
            self.erodeElement = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
            self.dilateElement = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        
        # Apply morphological operations to cleanup the image noise:
        imgth = cv2.erode(imgth, self.erodeElement)
        imgth = cv2.dilate(imgth, self.dilateElement)

        # Detect objects by finding contours:
        contours, hierarchy = cv2.findContours(imgth, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        str += "N={} ".format(len(contours))

        #Filter Contours by...
        output = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            #Width
            if w < self.min_width or w > self.max_width:
                continue
            #Height
            if h < self.min_height or h > self.max_height:
                continue
            area = cv2.contourArea(contour)
            #Area
            if area < self.min_area:
                continue
            #Perimeter
            if cv2.arcLength(contour, True) < self.min_perimeter:
                continue
            hull = cv2.convexHull(contour)
            solid = 100 * area / cv2.contourArea(hull)
            #Solidity
            if solid < self.solidity[0] or solid > self.solidity[1]:
                continue
            #Vertex Count
            if len(contour) < self.min_vertices or len(contour) > self.max_vertices:
                continue
            ratio = float(w) / h
            #W / H ratio
            if ratio < self.min_ratio or ratio > self.max_ratio:
                continue
            output.append(contour)

        # Display any results requested by the users:
        if outimg is not None and outimg.valid():
            if outimg.width == res_w * 2: jevois.pasteGreyToYUYV(imgth, outimg, res_w, 0)
            jevois.writeText(outimg, str, 3, res_h + 1, jevois.YUYV.White, jevois.Font.Font6x10)
            for contour in contours:
                box = cv2.boxPoints(cv2.minAreaRect(contour))
                point0 = box[-1]
                for point1 in box:
                    jevois.drawLine(outimg, int(point0[0]), int(point0[1]), int(point1[0]), int(point1[1]), 1, jevois.YUYV.LightPurple)
                    point0 = point1

        return output

    # ###################################################################################################
    ## Estimate 6D pose of each of the quadrilateral objects in hlist:
    def estimatePose(self, contour):
        object_points = np.array([[4, 31, 0.0], [5.936, 31.5, 0.0], [7.316, 26.175, 0.0], [5.375, 25.675, 0.0]])
        object_points = object_points.astype('float32')

        image_points = np.array(self.order_corners_clockwise(contour))
        image_points = image_points.astype('float32')

        ret, rotation, translation = cv2.solvePnP(object_points, image_points, self.camMatrix, self.distCoeffs)
        return rotation, translation
        
    # ###################################################################################################
    ## Send serial messages, one per object
    def sendAllSerial(self, w, h, hlist, rvecs, tvecs):
        idx = 0
        for c in hlist:
            # Compute quaternion: FIXME need to check!
            tv = tvecs[idx]
            axis = rvecs[idx]
            angle = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]) ** 0.5

            # This code lifted from pyquaternion from_axis_angle:
            mag_sq = axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]
            if (abs(1.0 - mag_sq) > 1e-12): axis = axis / (mag_sq ** 0.5)
            theta = angle / 2.0
            r = math.cos(theta)
            i = axis * math.sin(theta)
            q = (r, i[0], i[1], i[2])

            jevois.sendSerial("D3 {} {} {} {} {} {} {} {} {} {} FIRST".
                              format(np.asscalar(tv[0]), np.asscalar(tv[1]), np.asscalar(tv[2]),  # position
                                     self.owm, self.ohm, 1.0,                                     # size
                                     r, np.asscalar(i[0]), np.asscalar(i[1]), np.asscalar(i[2]))) # pose
            idx += 1

    # ###################################################################################################
    ## Draw all detected objects in 3D
    def drawDetections(self, outimg, hlist, rvecs = None, tvecs = None):
        # Show trihedron and parallelepiped centered on object:
        hw = self.owm * 0.5
        hh = self.ohm * 0.5
        dd = -max(hw, hh)
        i = 0
        empty = np.array([ (0.0), (0.0), (0.0) ])

        for obj in hlist:
            # skip those for which solvePnP failed:
            if np.array_equal(rvecs[i], empty):
                i += 1
                continue

            # Project axis points:
            axisPoints = np.array([ (0.0, 0.0, 0.0), (hw, 0.0, 0.0), (0.0, hh, 0.0), (0.0, 0.0, dd) ])
            imagePoints, jac = cv2.projectPoints(axisPoints, rvecs[i], tvecs[i], self.camMatrix, self.distCoeffs)

            # Draw axis lines:
            jevois.drawLine(outimg, int(imagePoints[0][0,0] + 0.5), int(imagePoints[0][0,1] + 0.5),
                            int(imagePoints[1][0,0] + 0.5), int(imagePoints[1][0,1] + 0.5),
                            2, jevois.YUYV.MedPurple)
            jevois.drawLine(outimg, int(imagePoints[0][0,0] + 0.5), int(imagePoints[0][0,1] + 0.5),
                            int(imagePoints[2][0,0] + 0.5), int(imagePoints[2][0,1] + 0.5),
                            2, jevois.YUYV.MedGreen)
            jevois.drawLine(outimg, int(imagePoints[0][0,0] + 0.5), int(imagePoints[0][0,1] + 0.5),
                            int(imagePoints[3][0,0] + 0.5), int(imagePoints[3][0,1] + 0.5),
                            2, jevois.YUYV.MedGrey)

            # Also draw a parallelepiped:
            cubePoints = np.array([ (-hw, -hh, 0.0), (hw, -hh, 0.0), (hw, hh, 0.0), (-hw, hh, 0.0),
                                    (-hw, -hh, dd), (hw, -hh, dd), (hw, hh, dd), (-hw, hh, dd) ])
            cu, jac2 = cv2.projectPoints(cubePoints, rvecs[i], tvecs[i], self.camMatrix, self.distCoeffs)

            # Round all the coordinates and cast to int for drawing:
            cu = np.rint(cu)

            # Draw parallelepiped lines:
            jevois.drawLine(outimg, int(cu[0][0,0]), int(cu[0][0,1]), int(cu[1][0,0]), int(cu[1][0,1]),
                            1, jevois.YUYV.LightGreen)
            jevois.drawLine(outimg, int(cu[1][0,0]), int(cu[1][0,1]), int(cu[2][0,0]), int(cu[2][0,1]),
                            1, jevois.YUYV.LightGreen)
            jevois.drawLine(outimg, int(cu[2][0,0]), int(cu[2][0,1]), int(cu[3][0,0]), int(cu[3][0,1]),
                            1, jevois.YUYV.LightGreen)
            jevois.drawLine(outimg, int(cu[3][0,0]), int(cu[3][0,1]), int(cu[0][0,0]), int(cu[0][0,1]),
                            1, jevois.YUYV.LightGreen)
            jevois.drawLine(outimg, int(cu[4][0,0]), int(cu[4][0,1]), int(cu[5][0,0]), int(cu[5][0,1]),
                            1, jevois.YUYV.LightGreen)
            jevois.drawLine(outimg, int(cu[5][0,0]), int(cu[5][0,1]), int(cu[6][0,0]), int(cu[6][0,1]),
                            1, jevois.YUYV.LightGreen)
            jevois.drawLine(outimg, int(cu[6][0,0]), int(cu[6][0,1]), int(cu[7][0,0]), int(cu[7][0,1]),
                            1, jevois.YUYV.LightGreen)
            jevois.drawLine(outimg, int(cu[7][0,0]), int(cu[7][0,1]), int(cu[4][0,0]), int(cu[4][0,1]),
                            1, jevois.YUYV.LightGreen)
            jevois.drawLine(outimg, int(cu[0][0,0]), int(cu[0][0,1]), int(cu[4][0,0]), int(cu[4][0,1]),
                            1, jevois.YUYV.LightGreen)
            jevois.drawLine(outimg, int(cu[1][0,0]), int(cu[1][0,1]), int(cu[5][0,0]), int(cu[5][0,1]),
                            1, jevois.YUYV.LightGreen)
            jevois.drawLine(outimg, int(cu[2][0,0]), int(cu[2][0,1]), int(cu[6][0,0]), int(cu[6][0,1]),
                            1, jevois.YUYV.LightGreen)
            jevois.drawLine(outimg, int(cu[3][0,0]), int(cu[3][0,1]), int(cu[7][0,0]), int(cu[7][0,1]),
                            1, jevois.YUYV.LightGreen)

            i += 1
            
    # ###################################################################################################
    ## Process function with no USB output
    def processNoUSB(self, inframe):
        # Get the next camera image (may block until it is captured) as OpenCV BGR:
        imgbgr = inframe.getCvBGR()
        h, w, chans = imgbgr.shape
        
        # Start measuring image processing time:
        self.timer.start()

        # Get a list of quadrilateral convex hulls for all good objects:
        hlist = self.detect(imgbgr)

        # Load camera calibration if needed:
        if not hasattr(self, 'camMatrix'): self.loadCameraCalibration(w, h)

        # Map to 6D (inverse perspective):
        (rvecs, tvecs) = self.estimatePose(hlist)

        # Send all serial messages:
        self.sendAllSerial(w, h, hlist, rvecs, tvecs)

        # Log frames/s info (will go to serlog serial port, default is None):
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
        outimg.require("output", res_w * 2, res_h + 12, jevois.V4L2_PIX_FMT_YUYV)
        jevois.paste(inimg, outimg, 0, 0)
        # jevois.drawFilledRect(outimg, 0, 0, w * 2, h + 12, jevois.YUYV.Black)
        jevois.drawFilledRect(outimg, 0, res_h, outimg.width, outimg.height-res_h, jevois.YUYV.Black)

        # Let camera know we are done using the input image:
        inframe.done()
        
        # Get a list of quadrilateral convex hulls for all good objects:
        contours = self.detect(imgbgr, outimg)

        # Load camera calibration if needed:
        if not hasattr(self, 'camMatrix'): self.loadCameraCalibration(res_w, res_h)

        contour_tracker = ContourTracker()
        center_target = contour_tracker.find_closest_contour(contours, (res_w, res_h))

        # Map to 6D (inverse perspective):
        rvecs, tvecs = self.estimatePose(center_target.right_rect)
        jevois.LINFO("({}, {}, {}) ({}, {}, {})".format(rvecs[0], rvecs[1], rvecs[2], tvecs[0], tvecs[1], tvecs[2]))

        # Send all serial messages:
        # self.sendAllSerial(w, h, hlist, rvecs, tvecs)

        # Draw all detections in 3D:
        # self.drawDetections(outimg, hlist, rvecs, tvecs)

        # Write frames/s info from our timer into the edge map (NOTE: does not account for output conversion time):
        fps = self.timer.stop()
        jevois.writeText(outimg, fps, 3, res_h-10, jevois.YUYV.White, jevois.Font.Font6x10)
    
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
        if self.distance(tl, right_most[0]) > self.distance(tl, right_most[1]):
            br, tr = right_most
        else:
            tr, br = right_most

        # return the coordinates in top-left, top-right, bottom-right, and bottom-left order
        return np.array([tl, tr, br, bl], dtype="float32")

    def distance(self, point0, point1):
        return math.sqrt(math.pow(point0[0] - point1[0], 2) + math.pow(point0[1] - point1[1], 2))


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

    def draw_target(self, frame):
        box_l = cv2.boxPoints(self.left_rect)
        box_r = cv2.boxPoints(self.right_rect)
        box_l = np.int0(box_l)
        box_r = np.int0(box_r)

        cv2.drawContours(frame, [box_l], 0, colors[0])
        cv2.drawContours(frame, [box_r], 0, colors[1])

    def draw_bounding_rect(self, frame):
        box_l = cv2.boxPoints(self.left_rect)
        box_r = cv2.boxPoints(self.right_rect)
        box_l = np.int0(box_l)
        box_r = np.int0(box_r)

        points = np.concatenate((box_l, box_r))
        x,y,w, h= cv2.boundingRect(points)

        retval = cv2.rectangle(frame, (x,y), (x + w, y + h), (0, 255,0 ), 1)


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

    def find_closest_contour(self, contours, frame_size, draw_targets=False):

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

        if draw_targets is not None:
            for target in remainder:
                target.draw_target(draw_targets)

        if len(remainder) > 0:
            remainder = min(remainder, key=lambda target: math.fabs(target.average_x - frame_size[1] / 2))

            if draw_targets is not None:
                remainder.draw_bounding_rect(draw_targets)
        else:
            remainder = None

        return remainder


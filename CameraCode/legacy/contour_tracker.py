import itertools as it
import math

import cv2
import numpy as np

has_networktable = True

try:
    from networktables import NetworkTables
except:
    has_networktable = False

from legacy import pipeline
from legacy.perspective_math import PerspectiveMath
from legacy.solve_extrinsics import SolveExtrinsics

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


if __name__ == "__main__":
    camera = cv2.VideoCapture(0)

    # img = "./vision examples/CargoAngledLine48in.jpg"
    # img = "./vision examples/CargoAngledDark48in.jpg"
    # img = "./vision examples/learn2.PNG"
    #img = "vision examples/CargoStraightDark48in.jpg"
    # img = "vision examples/CargoSideStraightDark60in.jpg"
    img = "./vision examples/42.25_8.25___16.25.jpg"

    source = cv2.imread(img)
    source = cv2.resize(source, (3280 // 2, 2464 // 2))
    grip = pipeline.FilterLines()
    # video = cv2.VideoCapture(convert)

    my_processor = ContourTracker()

    if has_networktable:
        NetworkTables.initialize(server='roborio-2974-frc.local')
        sd = NetworkTables.getTable('SmartDashboard')

    while True:

        # ret, source = camera.read()
        grip.process(source)
        frame_size = grip.get_mat_info_size
        perspective_math = PerspectiveMath(frame_size)
        solve_camera = SolveExtrinsics(frame_size)

        center_target = my_processor.find_closest_contour(grip.filter_contours_output, frame_size, source)

        if center_target is not None:
            '''
            robot_pose = perspective_math.calculate_robot_position(center_target.center_point_right,
                                                                   center_target.center_point_left)
            # print("%s,  %s" % (center_target.center_point_right[0] - frame_size[0] / 2, frame_size[1] / 2 - center_target.center_point_right[1]))
            # print("%s, %s" % (frame_size[0], frame_size[1]))
            # print("(%s, %s), %s degrees" % (robot_pose[0][0], robot_pose[0][1], robot_pose[1]))

            if has_networktable:
                sd.putNumber('x_value', robot_pose[0][0])
                sd.putNumber('z_value', robot_pose[0][1])
                sd.putNumber('angle', robot_pose[1])
            '''

            camera_extrinsics = solve_camera.calculate_extrinsics(center_target.right_rect)

            # print("(%s, %s, %s) degrees, (%s, %s, %s) inches" % (camera_extrinsics[0][0], camera_extrinsics[0][1],
            #                                                      camera_extrinsics[0][2], camera_extrinsics[1][0],
            #                                                      camera_extrinsics[1][1], camera_extrinsics[1][2]))
            print(camera_extrinsics[0])
            print(camera_extrinsics[1])
        else:
            print("There are no contours found")

        cv2.circle(source, (round(center_target.center_point_right[0]), round(center_target.center_point_right[1])), 2, (255, 50, 255), 2)
        # cv2.circle(source, (100, 100), 10, (255, 50, 255), 10)
        cv2.imshow("", source)

        if cv2.waitKey(500) & 0xFF == ord('q'):
            break

        # cv2.circle(source, (round(center_target.center_point_right[0]), round(center_target.center_point_right[1])), 10, (255, 50, 255))
        cv2.waitKey()

cv2.destroyAllWindows()

import itertools as it
import math

import cv2
import numpy as np

import pipeline
from perspective_math import PerspectiveMath

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


if __name__ == "__main__":

    source = "./vision examples/CargoStraightDark48in.jpg"
    img = cv2.imread(source)


    grip = pipeline.FilterLines()
   # video = cv2.VideoCapture(convert)

    my_processor = ContourTracker()
    pM = PerspectiveMath()

    while True:
    #    frame = video.grab()
        grip.process(img)

        frame_size = grip.get_mat_info_size
        try:
            center_target = my_processor.sort_contours(grip.filter_contours_output, frame_size)
            robot_pose = pM.calculate_robot_position(center_target.center_point_left, center_target.center_point_left, frame_size)
            print("(%s, %s), %s degrees"%(robot_pose[0][0], robot_pose[0][1], robot_pose[1]))
        except:
            print("No target found on the screen")

        cv2.imshow('frame', img)

        if cv2.waitKey(500) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

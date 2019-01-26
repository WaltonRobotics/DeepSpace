import cv2
import numpy as np
from scipy.spatial import distance as dist
from math import *

class SolveExtrinsics:
    def __init__(self, frame_size):
        self.frame_size = frame_size

        #Define 3D points of the right target with relation to and clockwise from the top left
        reference_y = 16.25
        object_points = [[4, 31, 0.0], [5.936, 31.5, 0.0],
                           [7.316, 26.175, 0.0],
                           [5.375, 25.675, 0.0]]
        object_points = np.array(object_points)
        self.object_points = object_points.astype('float32')
        # print(self.object_points)

    def calculate_extrinsics(self, contour):
        """
        :param contour: the right contour of the target
        :return: the extrinsic parameters of the camera as (rotation, translation)
        """
        image_points = np.array(self.order_corners_clockwise(contour))
        image_points = image_points.astype('float32')
        # print(image_points)
        ret, intrinsics, distortion, rotation, translation = cv2.calibrateCamera([self.object_points], [image_points],
                                                                                 self.frame_size, None, None)
        return rotation, translation

    @staticmethod
    def order_corners_clockwise(contour):
        """
        Sorts the corners of a contour clockwise from the top left point
        :param contour: the contour to sort the corners of
        :return: the sorted corners
        """
        #get corners of contour
        corners = cv2.boxPoints(contour)
        #sort corners by x value
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
        D = dist.cdist(tl[np.newaxis], right_most, "euclidean")[0]
        (br, tr) = right_most[np.argsort(D)[::-1], :]

        # return the coordinates in top-left, top-right, bottom-right, and bottom-left order
        return np.array([tl, tr, br, bl], dtype="float32")
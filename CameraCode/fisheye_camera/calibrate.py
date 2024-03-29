# https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0

import cv2
from cv2 import FileStorage

assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
import glob

CHECKERBOARD = (6, 9)
subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
_img_shape = None
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.
images = glob.glob('fisheye_calibration_images/*.jpg')
print(images)
gray = np.array([0, 0])

for fname in images:
    img = cv2.imread(fname)
    if _img_shape == None:
        _img_shape = img.shape[:2]
    else:
        assert _img_shape == img.shape[:2], "All images must share the same size."
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                             cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), subpix_criteria)
        imgpoints.append(corners)

        # If found, adddisplay the corners
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()
N_OK = len(objpoints)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

rms, _, _, _, _ = \
    cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs=rvecs,
        tvecs=tvecs,
        flags=calibration_flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
print("Found " + str(N_OK) + " valid images for calibration")
print("DIM=" + str(_img_shape[::-1]))
print("K=np.array(" + str(K.tolist()) + ")")
print("D=np.array(" + str(D.tolist()) + ")")

# You should replace these 3 lines with the output in calibration step
DIM = _img_shape[::-1]
K = K
D = D


def undistort(img, balance=0.0, dim2=None, dim3=None):
    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
    assert dim1[0] / dim1[1] == DIM[0] / DIM[
        1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1
    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)


    # filestaorgae= FileStorage("fisheye_undistorted.xml", cv2.FILE_STORAGE_WRITE_BASE64)
    # filestaorgae.write("K", K)
    # filestaorgae.write("scaledK", scaled_K)
    # filestaorgae.write("D", D)
    # filestaorgae.write("newK", new_K)
    # filestaorgae.write("dim3", dim3)
    # filestaorgae.write("map1", map1)
    # filestaorgae.write("map2", map2)
    #
    # filestaorgae.release()
    # print(K)
    # print(scaled_K)
    # print(D)
    # print(new_K)
    # print(dim3)
    print(map1)
    # print(map2)

    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img


if __name__ == '__main__':

    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    namedWindow = cv2.namedWindow("undistorted")

    max_balance = 100
    cv2.createTrackbar('balance', 'undistorted', 0, max_balance, lambda x: None)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        cv2.imshow('frame', frame)

        # Our operations on the frame come here
        balance = cv2.getTrackbarPos("balance", "undistorted") / max_balance
        frame = undistort(frame, balance=balance)

        # break
        # Display the resulting frame
        cv2.imshow('undistorted', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

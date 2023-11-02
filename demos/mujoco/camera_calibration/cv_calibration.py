import glob
import os

import cv2 as cv
import numpy as np


def get_intrinsics(chessboard_images_path, show=False):
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    images = glob.glob(os.path.join(chessboard_images_path, "*.png"), recursive=True)
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (7, 6), None)
        # If found, add object points, image points (after refining them)
        print(ret)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)
            # Draw and display the corners
            if show:
                cv.drawChessboardCorners(img, (7, 6), corners2, ret)
                cv.imshow("img", img)
                cv.waitKey(500)

    cv.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    return mtx


if __name__ == "__main__":
    path = "/home/goat/Documents/Master/SimulationFramework2/simulation/envs/chessboard_images"
    print(get_intrinsics(path))

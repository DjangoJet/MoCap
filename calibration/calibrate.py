import numpy as np
import cv2
import glob
import argparse
import matplotlib.pyplot as plt
import pickle


class StereoCalibration():
    def __init__(self):
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.criteria_cal = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((9*7, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:9, 0:7].T.reshape(-1, 2)*2

        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints_l = []  # 2d points in image plane.
        self.imgpoints_r = []  # 2d points in image plane.

        self.camerasCalibrate()
        self.stereoCalibrate()
        self.computeNewCameraMatrixs()
        self.computeProjectionMatrixs()

    def camerasCalibrate(self):
        images_right = glob.glob('RIGHT/*.jpg')
        images_left = glob.glob('LEFT/*.jpg')
        images_left.sort()
        images_right.sort()

        for i, _ in enumerate(images_right):
            img_l = cv2.imread(images_left[i])
            img_r = cv2.imread(images_right[i])

            gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret_l, corners_l = cv2.findChessboardCorners(gray_l, (9, 7), None)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, (9, 7), None)

            # If found, add object points, image points (after refining them)
            self.objpoints.append(self.objp)

            if ret_l is True:
                _ = cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1), self.criteria)
                self.imgpoints_l.append(corners_l)

                # Draw and display the corners
                ret_l = cv2.drawChessboardCorners(img_l, (9, 7), corners_l, ret_l)
                # plt.imshow(img_l)
                # plt.title(images_left[i])
                # plt.show()

            if ret_r is True:
                _ = cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), self.criteria)
                self.imgpoints_r.append(corners_r)

                # Draw and display the corners
                ret_r = cv2.drawChessboardCorners(img_r, (9, 7), corners_r, ret_r)
                # plt.imshow(img_r)
                # plt.title(images_right[i])
                # plt.show()
            self.img_shape = gray_l.shape[::-1]

        # print(type(img_shape))
        _, self.M_L, self.d_L, _, _ = cv2.calibrateCamera(self.objpoints, self.imgpoints_l, self.img_shape, None, None)
        _, self.M_R, self.d_R, _, _ = cv2.calibrateCamera(self.objpoints, self.imgpoints_r, self.img_shape, None, None)

    def stereoCalibrate(self):
        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        flags |= cv2.CALIB_ZERO_TANGENT_DIST

        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        _ , self.M_L, self.d_L, self.M_R, self.d_R, self.R, self.T, self.E, self.F = cv2.stereoCalibrate(self.objpoints, self.imgpoints_l,self.imgpoints_r, self.M_L, self.d_L, self.M_R, self.d_R, self.img_shape, criteria=stereocalib_criteria, flags=flags)
        self.R_L, self.R_R, self.P_L, self.P_R, self.Q, self.roi_L, self.roi_R = cv2.stereoRectify(
                        self.M_L,
                        self.d_L,
                        self.M_R,
                        self.d_R,
                        self.img_shape,
                        self.R,
                        self.T)


    def computeNewCameraMatrixs(self):
        w, h = self.img_shape
        self.new_M_L, _ = cv2.getOptimalNewCameraMatrix(self.M_L, self.d_L,(w,h),1,(w,h))
        self.new_M_R, _ = cv2.getOptimalNewCameraMatrix(self.M_R, self.d_R,(w,h),1,(w,h))

    def computeProjectionMatrixs(self):
        R_T = np.append(self.R, self.T, axis = 1)
        self.P_L = np.dot(self.new_M_L, R_T)
        self.P_R = np.dot(self.new_M_R, np.array([(1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0)]))

    def saveToFile(self):
        params = {"M_L": self.M_L, "d_L": self.d_L, "M_R": self.M_R, "d_R": self.d_R, "R": self.R, "T": self.T, "E": self.E, "F": self.F, "R_L": self.R_L, "R_R": self.R_R, "P_L": self.P_L, "P_R": self.P_R, "Q": self.Q, "roi_L": self.roi_L, "roi_R": self.roi_R}
        with open('params', 'wb') as f:
            pickle.dump(params, f)




if __name__ == '__main__':
    stereoCalibration = StereoCalibration()
    stereoCalibration.saveToFile()
    # projection matrix
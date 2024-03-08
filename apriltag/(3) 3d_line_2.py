# 인식된 AprilTag에 Camera Calibration 값을 적용해 3D line 그리는 코드
# 본 코드에는 실행시킬 때마다 Calibration을 수행하도록 구성되어있으니 따로 저장해놓은 Calibration 값을 가져오는 코드는 (5)번 코드를 참고할 것
# 3D
# 여기서부터는 Queue가 자주 사용되므로 관련 자료조사 및 정리해둘 것 line을 그리는 코드의 경우 내가 새로 만들어낸 방식일 뿐, 원래 사용되는 방식과 다르니까 관련 자료를 찾아볼 것

import numpy as np
from queue import Queue
import cv2
import glob
import apriltag
import time

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

wc = 10
hc = 7

draw_img = cv2.imread('tag_sample/5.jpg')

q = Queue()

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((wc*hc, 3), np.float32)
objp[:, :2] = np.mgrid[0:wc, 0:hc].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
c = []


def draw(img, corners, imgpts):
    corner = list(map(int, tuple(corners[0].ravel())))
    img = cv2.line(img, corner, imgpts[0], (255,0,0), 5)
    img = cv2.line(img, corner, imgpts[1], (0,255,0), 5)
    img = cv2.line(img, corner, imgpts[2], (0,0,255), 5)
    return img


def camera_calibration():
    images = glob.glob('cali_sample/*.jpg')

    for fname in images:

        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (wc, hc), None)
        

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (10, 10), (-1, -1), criteria)
            #print(corners2)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (wc, hc), corners2, ret)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        #print(dist)
        print(mtx)
            
        q.put(mtx)
        q.put(dist)


def point_draw():
    img = draw_img
    
    mtx = q.get()
    dist = q.get()
    
    ##########################################
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detector = apriltag.Detector()
    result = detector.detect(gray)
    
    for r in result:
        corners = r.corners
        #print(corners)
        
        corners = corners.tolist()
        c.append(corners[2])
        c.append(corners[3])
        
        corners[2] = c[1]
        corners[3] = c[0]
        
        corners = np.array(corners)
        
        corners = np.reshape(corners,(4,1,2))
        #print(c)
        #print(corners)
        
        wc = 2
        hc = 2
        
        ##########################################
        
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1)
        #print(newcameramtx)
        print(mtx)
        
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        
        objp2 = np.zeros((wc*hc, 3), np.float32)
        objp2[:, :2] = np.mgrid[0:wc, 0:hc].T.reshape(-1, 2)
        
        axis = np.float32([[1, 0, 0], [0, 1, 0], [0, 0, -1]]).reshape(-1, 3)
        
        # Find the rotation and translation vectors.
        _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp2, corners, mtx, dist)

        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        print(imgpts)
        imgpts = list(map(int, tuple(imgpts[0].ravel()))), list(map(int, tuple(imgpts[1].ravel()))), list(map(int, tuple(imgpts[2].ravel())))
        
        img = draw(dst, corners, imgpts)
        ##########################################

        cv2.imshow('result', img)
    
    

if __name__ == "__main__":
    camera_calibration()
    point_draw()
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()
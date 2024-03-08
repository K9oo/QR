# 카메라의 위치를 x,y,z 값으로 출력하고, 그 값을 이용해 Tag와의 거리를 계산하는 코드
# 본 코드도 실행 시 calibration이 수행되는 코드이므로 수정 필요
# math 라이브러리 설치 필요

import numpy as np
from queue import Queue
import cv2
import glob
import apriltag
import time
import math

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

wc = 10
hc = 7

tag_size = 0.037
chess_size = 0.033

draw_img = cv2.imread('tag_sample/30.jpg')

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
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (wc, hc), corners2, ret)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            
    q.put(mtx)
    q.put(dist)


def point_draw():
    img = draw_img
    
    mtx = q.get()
    dist = q.get()
    #print(mtx)
    #print(dist)
    
    ##########################################
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detector = apriltag.Detector()
    result = detector.detect(gray)
    
    for r in result:
        corners = r.corners
        corners = corners.tolist()
        c.append(corners[2])
        c.append(corners[3])
        
        corners[2] = c[1]
        corners[3] = c[0]
        
        corners = np.array(corners)
        #print(corners.shape)
        corners = np.reshape(corners,(4,1,2))
        
        wc = 2
        hc = 2
        
        ##########################################
        
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1)

        
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        
        objp2 = np.zeros((wc*hc, 3), np.float32)
        objp2[:, :2] = np.mgrid[0:wc, 0:hc].T.reshape(-1, 2)*tag_size
        #objp2 = objp2*tag_size
        
        axis = np.float32([[tag_size, 0, 0], [0, tag_size, 0], [0, 0, -tag_size]]).reshape(-1, 3)
        
        # Find the rotation and translation vectors.
        _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp2, corners, mtx, dist)
        
        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        imgpts = list(map(int, tuple(imgpts[0].ravel()))), list(map(int, tuple(imgpts[1].ravel()))), list(map(int, tuple(imgpts[2].ravel())))
        img = draw(img, corners, imgpts)
        
        corners = (w, h)
        Rt = cv2.Rodrigues(rvecs)[0]
        #R = Rt.transpose()    # DCM
        #cameraPosition = -R*np.matrix(tvecs)
        cameraPosition = -np.matrix(Rt).T*np.matrix(tvecs)
        distance = math.sqrt(cameraPosition[0]**2+cameraPosition[1]**2+cameraPosition[2]**2)*100
        print(cameraPosition)
        print(round(distance,1))
        
        #roll = atan2(-R[2][1], R[2][2])
        #pitch = asin(R[2][0])
        #yaw = atan2(-R[1][0], R[0][0])
        
        ##########################################

        cv2.imshow('result', img)
    
    

if __name__ == "__main__":
    camera_calibration()
    point_draw()
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()
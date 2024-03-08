# 비행하면서 인식한 AprilTag의 정보를 토대로 tello의 현재 좌표 값과 Tag와의 거리를 추정하는 코드
# 주석 처리되어있는 'print( ), file=파일명)' 부분을 주석 해제 시 txt파일로 저장가능

from djitellopy import Tello
from queue import Queue
import numpy as np
import threading
import cv2
import time
import apriltag
import math
import sys

######################################################################
width = 320  # WIDTH OF THE IMAGE
height = 240  # HEIGHT OF THE IMAGE
startCounter =0   #  0 FOR FIGHT 1 FOR TESTING
######################################################################

# CONNECT TO TELLO
me = Tello()
me.connect()
me.for_back_velocity = 0
me.left_right_velocity = 0
me.up_down_velocity = 0
me.yaw_velocity = 0
me.speed = 0

print(me.get_battery())

me.streamoff()
me.streamon()

speed = 10
S = 5
FPS = 30

q = Queue()
c = []

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

###################################################
img_cali = np.load('/home/pi/Desktop/Lee/AprilTag_tello/mtx,dist/real_cali_2.npz')

mtx = img_cali['m']
dist = img_cali['d']

mtx = np.array(mtx, dtype=np.float32)
dist = np.array(dist, dtype=np.float32)

img_cali.close()

####################################################
#unit: 'mm'
#tag_size = 0.036

#tag size x2
#tag_size = 0.072

#flight test tag size
tag_size = 53

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
            
def draw_line(myFrame, corners, imgpts):
        corner = list(map(int, tuple(corners[0].ravel())))
        myFrame = cv2.line(myFrame, corner, imgpts[0], (255,0,0), 5)
        myFrame = cv2.line(myFrame, corner, imgpts[1], (0,255,0), 5)
        myFrame = cv2.line(myFrame, corner, imgpts[2], (0,0,255), 5)
        return myFrame
    
    
class Test():
    def Detection(self):
        global mtx, dist
        
        while True:
            try:
                frame_read = me.get_frame_read()
                myFrame = frame_read.frame
                
                gray = cv2.cvtColor(myFrame, cv2.COLOR_BGR2GRAY)
                
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
                    
                    wc = 2
                    hc = 2
                    
                    ##########################################                        
                    h, w = gray.shape[:2]
                    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1)
                    #print(mtx)
                    
                    #dst = cv2.undistort(myFrame, mtx, dist, None, newcameramtx)
                    
                    objp = np.zeros((wc*hc, 3), np.float32)
                    objp[:, :2] = np.mgrid[0:wc, 0:hc].T.reshape(-1, 2)*tag_size
                    #objp = objp*tag_size
                    
                    axis = np.float32([[tag_size, 0, 0], [0, tag_size, 0], [0, 0, -tag_size]]).reshape(-1, 3)
                    
                    # Find the rotation and translation vectors.
                    _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, mtx, dist)
                    
                    # project 3D points to image plane
                    imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
                    imgpts = list(map(int, tuple(imgpts[0].ravel()))), list(map(int, tuple(imgpts[1].ravel()))), list(map(int, tuple(imgpts[2].ravel())))
                    myFrame = draw_line(myFrame, corners, imgpts)
                    
                    corners = (w, h)
                    Rt = cv2.Rodrigues(rvecs)[0]
                    cameraPosition = -np.matrix(Rt).T*np.matrix(tvecs)
                    
                    distance = math.sqrt(cameraPosition[0]**2+cameraPosition[1]**2+cameraPosition[2]**2)*0.1
                    #print(cameraPosition[2]*0.1)
                    print(round(distance,2))
                    
                    #f_d = open('22.11.28_dist_esti.txt','a')
                    #print(round(distance,2), file=f_d)
                    #f_d.close
                    
                    ID = r.tag_id
                    if ID == 0:
                        x = 0 + cameraPosition[0]*0.1
                        y = 0 + cameraPosition[1]*0.1
                        z = 0 + cameraPosition[2]*0.1
                        
                    if 0 < ID <= 4:
                        x = (ID-1)*30 + cameraPosition[0]*0.1
                        y = 30 + cameraPosition[1]*0.1
                        z = 0 + cameraPosition[2]*0.1
                        
                    if ID > 4:
                        x = (ID-5)*30 + cameraPosition[0]*0.1
                        y = 60 + cameraPosition[1]*0.1
                        z = 0 + cameraPosition[2]*0.1
                    
                    coordinate = x, y, z
                    #print(coordinate)
                
                    #f_c = open('22.11.28_coordinate.txt','a')
                    #print(coordinate, file=f_c)
                    #f_c.close
                    
                    c.clear()
                                
                cv2.imshow('result', myFrame)
                
                if cv2.waitKey(1) & 0xFF == 13: #enter
                    break

            except Exception as ex:
                print("def_Detection error.", ex)
            
            
    def myStart(self):
        self.Detection_Thread = threading.Thread(target=self.Detection)

        self.Detection_Thread.daemon = True

        self.Detection_Thread.start()
        
        pass


    def myStop(self):
        print('myStop')
        if cv2.waitKey(1) & 0xFF == ord('z'):
           sys.exit()
        pass

def Flight():
    time.sleep(3)
    print("! Tack off")
    me.tackoff()
    print("! move up")
    me.move_up(60)
    time.sleep(3)
    print("! move forward")
    me.move_forward(100)
    time.sleep(3)
    me.move_right(30)
    time.sleep(3)
    me.move_right(50)
    time.sleep(3)
    me.move_up(60)
    time.sleep(3)
    me.move_left(30)
    time.sleep(3)
    me.move_left(50)
    time.sleep(3)
    print("! Landing")
    me.move_back(60)
    time.sleep(3)
    me.land()
    time.sleep(3)
    

if __name__ == "__main__":
    print("Start program....")
    test = Test()
    test.myStart()
    #Flight()
    x = input('Enter Any key to stop')
    test.myStop()
    print("\n\nEnd of Program\n")
    
    
    


# QR code와 AprilTag가 부착된 '창고재고관리 드론 시스템' 두 번째 final 코드(졸업 논문으로 사용한 최종 코드)
# QR code 인식과 AprilTag 인식을 실시간으로 이루어지도록 하기 위해 Multi-Threading 기법 적용
# 본 코드에서는 deque도 사용하고 있으니, 관련 라이브러리 install 필요

from djitellopy import Tello
from queue import Queue
from collections import deque
import numpy as np
import threading
import cv2
import time
import apriltag
import math
import sys
import pyzbar.pyzbar as pyzbar


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
deq = deque()
c = []

QRcode_list = []
Past_QRcode = []
ID_list = []
Past_ID = []
check_list=['CJU4090301', 'CJU4090302', 'CJU4090303', 'CJU4090401', 'CJU4090402', 'CJU4090403', 'CJU4090501', 'CJU4090502', 'CJU4090503']

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

###################################################
img_cali = np.load('/home/pi/Desktop/Lee/AprilTag_tello/mtx,dist/real_cali_2.npz')

mtx = img_cali['m']
dist = img_cali['d']

mtx = np.array(mtx, dtype=np.float32)
dist = np.array(dist, dtype=np.float32)

img_cali.close()

####################################################

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)         
def draw_line(myFrame, corners, imgpts):
        corner = list(map(int, tuple(corners[0].ravel())))
        myFrame = cv2.line(myFrame, corner, imgpts[0], (255,0,0), 5)
        myFrame = cv2.line(myFrame, corner, imgpts[1], (0,255,0), 5)
        myFrame = cv2.line(myFrame, corner, imgpts[2], (0,0,255), 5)
        return myFrame
    
    
class Test():
    def AprilTag(self):
        global mtx, dist, stop_threads
        
        while True:
            try:
                frame_read = me.get_frame_read()
                myFrame = frame_read.frame
                
                gray = cv2.cvtColor(myFrame, cv2.COLOR_BGR2GRAY)
                decoded = pyzbar.decode(gray)
                q.put(myFrame)
                
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
                    ID = r.tag_id
                    #q.put(ID)

                    # unit: 'mm'
                    tag_size = 146
                    #tag_size = 100
                    
                    objp = np.zeros((wc*hc, 3), np.float32)
                    objp[:, :2] = np.mgrid[0:wc, 0:hc].T.reshape(-1, 2)*tag_size
                    #print(tag_size)
                    print("number: "+str(ID))
                    
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
                    print(cameraPosition*0.1)
                    #q.put(round(distance,2))
                    print("distance: "+str(round(distance,2))+"cm")
                    deq.appendleft(distance)
                    
                    f_i = open('22.12.12_id6.txt','a')
                    print("number: "+str(ID), file=f_i)
                    f_i.close
                    
                    f_d = open('22.12.12_dist_esti6.txt','a')
                    print(round(distance,2), file=f_d)
                    f_d.close
                    
                    # drone first position: (7.3, 14.6, 0)
                    if ID == 0:
                        x = 0 + cameraPosition[0]*0.1
                        y = 0 - cameraPosition[1]*0.1
                        z = 0 - cameraPosition[2]*0.1

                    if ID == 1:
                        x = 0 + cameraPosition[0]*0.1
                        y = 70 - cameraPosition[1]*0.1
                        z = 0 -cameraPosition[2]*0.1

                    if 1 < ID <= 3:
                        x = 95.2 + cameraPosition[0]*0.1
                        y = 72.7+(ID-2)*68.2 - cameraPosition[1]*0.1
                        z = 41.6 -cameraPosition[2]*0.1
                    
                    if ID == 4:
                        x = 200 + cameraPosition[0]*0.1
                        y = 70 - cameraPosition[1]*0.1
                        z = 0 -cameraPosition[2]*0.1
                        
                    
                    coordinate = x, y, z
                    deq.appendleft(coordinate)
                    #print(coordinate)
                
                    f_c = open('22.12.12_coordinate6.txt','a')
                    print(coordinate, file=f_c)
                    f_c.close
                    
                    c.clear()
                    
                cv2.imshow('result', myFrame)
                cv2.waitKey(1)
            
            except Exception as ex:
                print("def_AprilTag error.", ex)
                
            if stop_threads == True:
                break 
                
                
    
    def QRcode(self):
        while True:
            try:
                #while deq:
                    #myFrame = deq.pop()
                myFrame = q.get()
                gray = cv2.cvtColor(myFrame, cv2.COLOR_BGR2GRAY)
                decode = pyzbar.decode(gray)
                
                barcode_data = ""
                global QRcode_list, stop_threads
                
                for d in decode:
                    x, y, w, h = d.rect

                    barcode_data = d.data.decode("ascii")
                    barcode_type = d.type
                    
                    QRcode_list.append(f'{barcode_data:.12}')

                    cv2.rectangle(myFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    text = '%s (%s)' % (barcode_data, barcode_type)
                    cv2.putText(myFrame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
                
                for i in QRcode_list:
                    if i not in Past_QRcode:
                        Past_QRcode.append(i)
            
            except Exception as ex:
                print("def_QRcode error.", ex)
                
            if stop_threads == True:
                break 

    
    def myStart(self):
        self.AprilTag_Thread = threading.Thread(target=self.AprilTag)
        self.QRcode_Thread = threading.Thread(target=self.QRcode)
        
        self.AprilTag_Thread.daemon = True
        self.QRcode_Thread.daemon = True

        self.AprilTag_Thread.start()
        self.QRcode_Thread.start()
        

    def myStop(self):
        print('myStop')
        sys.exit()
        thread.join


def Flight():
    global stop_threads
    time.sleep(3)
    print("! Take off")
    me.takeoff()
    #print("! move up")
    #me.move_up(60)
    time.sleep(3)
    
    me.move_right(90)
    time.sleep(3)
    
    me.move_back(30)
    time.sleep(3)

    me.move_right(20)
    time.sleep(3)
    
    me.move_up(50)
    time.sleep(3)

    me.move_left(20)
    time.sleep(3)
    
    me.move_up(20)
    time.sleep(3)
    
    me.move_right(20)
    time.sleep(3)
    
    me.move_right(100)
    time.sleep(3)
    
    print("! Landing")
    me.land()
    time.sleep(3)
    stop_threads = True
    
    return stop_threads



if __name__ == "__main__":
    stop_threads = False
    print("Start program....")
    test = Test()
    test.myStart()
    #Flight()
    
    x = input('Enter Any key to stop')
    if cv2.waitKey(1) & 0xFF == 13: #enter
        test.myStop()
        
    print(Past_QRcode)
    for i in check_list:
        if i not in Past_QRcode:
            print('The missing product is %s.' % i)
            continue
        
    test.myStop()
    print("\n\nEnd of Program\n")
    
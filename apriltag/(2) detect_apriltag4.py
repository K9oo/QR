# Tello 카메라를 통한 실시간 AprilTag 인식

from djitellopy import Tello
import numpy as np
import threading
import cv2
import time
import apriltag
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

class Test:

    def Detection(self):
        while True:
            try:
                frame_read = me.get_frame_read()
                myFrame = frame_read.frame

                gray = cv2.cvtColor(myFrame, cv2.COLOR_BGR2GRAY)
               
                #global Detect_data_April
                
                detector = apriltag.Detector()
                result = detector.detect(gray)
                #Detect_data_April.append(result)
                
                for r in result:
                    for idx in range(len(r.corners)):
                        cv2.line(myFrame, tuple(r.corners[idx-1, :].astype(int)), tuple(r.corners[idx, :].astype(int)), (0, 255, 0), 3)
                    
                    cv2.putText(myFrame, str(r.tag_id), org=(r.corners[0,0].astype(int)+10, r.corners[0,1].astype(int)+10),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255), thickness=2)           
                    
                    
                cv2.imshow("MyResult", myFrame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
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
            cv2.destroyAllWindows()
        pass


if __name__ == "__main__":
    print("Start program....")
    test = Test()
    test.myStart()
    x = input('Enter Any key to stop')
    test.myStop()
    print("\n\nEnd of Program\n")

# QR code가 부착된 '창고재고관리 드론 시스템' 첫 번째 final 코드
# Multi-Threading 관련 코드도 섞여있으니 필요시 추가적으로 공부 및 자료조사 진행할 것

from djitellopy import Tello
from queue import Queue
import threading
import cv2
import time
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

QRcode_list = []
Past_QRcode = []
check_list=['CJU4090201', 'CJU4090202', 'CJU4090203', 'CJU4090204', 'CJU4090301', 'CJU4090302', 'CJU4090303', 'CJU4090304']

class Test:

    def Video(self):
        while True:
            try:
                frame_read = me.get_frame_read()
                myFrame = frame_read.frame

                gray = cv2.cvtColor(myFrame, cv2.COLOR_BGR2GRAY)

                decoded = pyzbar.decode(gray)
                q.put(decoded)

                for d in decoded:
                    x, y, w, h = d.rect
                    
                    barcode_data = d.data.decode("utf-8")
                    barcode_type = d.type

                    cv2.rectangle(myFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    text = '%s (%s)' % (barcode_data, barcode_type)
                    cv2.putText(myFrame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)

                cv2.imshow("MyResult", myFrame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            except Exception as ex:
                print("def_Video error.", ex)


    def list_check(self):
        while True:
            try:
                decodedObjects = q.get()
                aux = ""
                global QRcode_list

                for obj in decodedObjects:
                    aux += obj.data.decode('ascii')
                    QRcode_list.append(f'{aux:.12}')

                for i in QRcode_list:
                    if i not in Past_QRcode:
                        Past_QRcode.append(i)


            except Exception as ex:
                print("def_Active error.", ex)


    def myStart(self):
        self.Video_Thread = threading.Thread(target=self.Video)
        self.Active_Thread = threading.Thread(target=self.list_check)

        self.Video_Thread.daemon = True
        self.list_check_Thread.daemon = True

        self.Video_Thread.start()
        self.list_check_Thread.start()
        pass


    def myStop(self):
        print('myStop')
        if cv2.waitKey(1) & 0xFF == ord('z'):
            cv2.destroyAllWindows()
        pass

def Flight():
    time.sleep(3)
    print("! Take off")
    me.takeoff()
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
    print(Past_QRcode)
    for i in Past_QRcode:
        if i not in check_list:
            print('The missing product is %s.' % i)
            continue
    test.myStop()
    print("\n\nEnd of Program\n")

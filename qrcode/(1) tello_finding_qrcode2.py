# 인식되는 QR code ID에 따라 tello 제어명령 입력되는 코드
# QR code 인식 코드는 #으로 선을 그어 구별해두었으니 참고할 것
# pyzbar 라이브러리 필요: pip install pyzbar

from djitellopy import Tello
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
S = 60
FPS = 30

while True :
    # GET THE IMGAE FROM TELLO
    frame_read = me.get_frame_read()
    myFrame = frame_read.frame
    img = cv2.resize(myFrame, (width, height))

   ##########################################################################
    # QRcode 인식 코드
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    decoded = pyzbar.decode(gray)

    for d in decoded:
        x, y, w, h = d.rect

        barcode_data = d.data.decode("utf-8")
        barcode_type = d.type

        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)

        text = '%s (%s)' % (barcode_data, barcode_type)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
    #########################################################################

    cv2.imshow("MyResult", img)

    decodedObjects = pyzbar.decode(myFrame)
    aux = "";

    for obj in decodedObjects:

        aux += obj.data.decode('ascii')

    if aux == "first":
        print('first')
        # 원하는 tello 제어 명령 입력
        # me.takeoff()
        # time.sleep(3)
        # me.move_up(80)
        # time.sleep(3)
        # me.move_forward(100)
        # time.sleep(3)


    elif aux == "second":
        print('second')

    elif aux == "third":
        print('third')

    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        break

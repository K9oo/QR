# 코드 (1)의 응용 예시
# 'Rise'라는 ID를 가진 QR code를 인식했을 시 take off

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
        # Makes the QR code content into a String
        aux += obj.data.decode('ascii')

    if aux == "Rise":
        print('Rise')
        me.takeoff()
        me.move_up(70)

    elif aux == "Land":
        print('Land')
        me.land()
        time.sleep(5)

    elif aux == "Curve":
        print('Curve')
        me.move_forward(100)
        time.sleep(3)
        me.rotate_clockwise(90)
        time.sleep(3)
        me.move_forward(100)
        time.sleep(3)
        me.rotate_clockwise(90)
        time.sleep(3)
        me.move_forward(100)
        time.sleep(3)
        me.rotate_clockwise(90)
        time.sleep(3)
        me.move_forward(100)
        time.sleep(3)
        me.rotate_clockwise(90)
        time.sleep(3)

    elif aux == "Square":
        print('Square')
        me.move_right(70)
        time.sleep(3)
        me.move_up(70)
        time.sleep(3)
        me.move_left(70)
        time.sleep(3)
        me.move_down(70)
        time.sleep(3)

    elif aux == "Flip":
        print('Flip')
        me.flip_right()
        time.sleep(3)
        me.flip_left()
        time.sleep(3)
        me.flip_right()
        time.sleep(3)
        me.flip_left()
        time.sleep(3)


    # WAIT FOR THE 'Q' BUTTON TO STOP
    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        break
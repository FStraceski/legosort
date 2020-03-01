import numpy as np
import cv2
import IPython.display
import PIL.Image
from io import BytesIO
import imutils
import serial
import time

ser = serial.Serial('/dev/serial0', 115200, timeout=0)

cap = cv2.VideoCapture(0)
# d1 = IPython.display.display("Your image will be displayed here", display_id=1)

counter = 0

lower_red = np.array([30, 150, 50])
upper_red = np.array([255, 255, 180])



while (True):

    ret, frame = cap.read()
    brick = ser.read()

    if brick == b'1':
        elapsed_time = 0
        start_time = time.time()
        while elapsed_time < 3.5:
            elapsed_time = time.time() - start_time
            ret, frame = cap.read()

        # Our operations on the frame come here
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        #rgb = cv2.cvtColor(frame, cv2.COLOR_HSV2RGB)
        hsv = imutils.resize(hsv, height=100, width=100, inter=cv2.INTER_LINEAR)
        hsv = hsv[:, 20:95, :]
        hsv = imutils.resize(hsv, height=32, width=32, inter=cv2.INTER_LINEAR)
        #image = array_to_image(rgb)

        mask = cv2.inRange(hsv, lower_red, upper_red)
        res = cv2.bitwise_and(hsv, hsv, mask=mask)

        # Display the resulting frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        #counter += 1

        prediction = b''

        print(np.average(res))
        if np.average(res) > .3:
            prediction = b'0'
            print("Predicting: RED")
        else:
            prediction = b'1'
            print("Predicting: NOT RED")

        ser.write(prediction)

        # np.save("red/red" + str(counter), hsv)
        # print("saved image number " + str(counter))

# When everything done, release the capture
cap.release()
ser.close()

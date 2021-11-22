import sys
import os
import dlib
import glob
import numpy as np
import cv2
import math
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)
GPIO.setup(23,GPIO.OUT)
GPIO.setup(24,GPIO.OUT)



cap = cv2.VideoCapture(-1)
fourcc = cv2.VideoWriter_fourcc(*'XVID')

out = cv2.VideoWriter('output.avi',fourcc, 20.0, (1280, 720))

predictor_path = '/home/abdulqader/Documents/eecs373/thermal_turret/cv/Videos/shape_predictor_81_face_landmarks.dat'

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)
pTime = 0
while(cap.isOpened()):
    ret, frame = cap.read()
    frame = cv2.resize(frame,(400,400),fx=0,fy=0, interpolation = cv2.INTER_CUBIC)
    dets = detector(frame, 0)
    my_dict = {"69": [0,0], "73": [0,0], "74": [0,0],"24": [0,0],"19": [0,0],"75": [0,0]};
    nums = [69,73,74,24,19,75]
    for k, d in enumerate(dets):
        shape = predictor(frame, d)
        landmarks = np.matrix([[p.x, p.y] for p in shape.parts()])
        
        for num in range(shape.num_parts):
            x = shape.parts()[num].x
            y = shape.parts()[num].y
            
            if str(num) in my_dict:
                my_dict[str(num)]=[x,y]
                cv2.circle(frame, (x, y), 3, (0,255,0), -1)
                z = str(x) + ',' + str(y)
                cv2.putText(frame, str(num), (x, y), cv2.FONT_HERSHEY_PLAIN,
                        1, (0, 255, 0), 1)

    center = (math.ceil(frame.shape[1]/2)-1,math.ceil(frame.shape[0]/2)-1)
 
    cTime = time.time()
    fps = 1/ (cTime-pTime)
    pTime = cTime
    
    cv2.putText(frame, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_PLAIN,3,  (0, 255, 0), 3)
    if (my_dict["75"][0] < center[0]) and (my_dict["69"][1] < center[1]) and (my_dict["19"][1] > center[1]) and (my_dict["74"][0] > center[0]):
        print("shoot")
        cv2.rectangle(frame, (math.ceil(frame.shape[1] / 2) - 40, math.ceil(frame.shape[0] / 2) - 40),
                        (math.ceil(frame.shape[1] / 2) + 40, math.ceil(frame.shape[0] / 2) + 40), (255, 0, 0), 3)



    #4 pins for direction, 21 right, 22 left, 23 up, 24 down   


    #19 corresponds to lower-left (from camera pov) 
    #24 corresponds to lower-right
    #69 corresponds to upper-left
    #73 corresponds to upper-right

    if(my_dict["19"][0] > center[0]):
        #We're to the left of the left
        GPIO.output(21, True)
        print("Trying to move RIGHT")
    else:
        GPIO.output(21, False)
    if(my_dict["24"][0] < center[0]):
        #We're to the right of the right
        GPIO.output(22, True)
        print("Trying to move LEFT")
    else:
        GPIO.output(22, False)
    if(my_dict["24"][1] < center[1]):
        #We're below the bottom
        GPIO.output(23, True)
        print("Trying to move UP")
    else:
        GPIO.output(23, False)
    if(my_dict["73"][1] >  center[1]):
        #We're above the top
        GPIO.output(24, True)
        print("Trying to move DOWN")
    else:
        GPIO.output(24,False)

    out.write(frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("q pressed")
        break
    cv2.rectangle(frame, (math.ceil(frame.shape[1]/2)-1,math.ceil(frame.shape[0]/2)-1), (math.ceil(frame.shape[1]/2)+1,math.ceil(frame.shape[0]/2)+1), (255,0,0), -1)
cap.release()
out.release()

cv2.destroyAllWindows()




import sys
import os
import dlib
import glob
import numpy as np
import cv2
import math


cap = cv2.VideoCapture(1)
fourcc = cv2.VideoWriter_fourcc(*'XVID')

out = cv2.VideoWriter('output.avi',fourcc, 20.0, (1280, 720))

predictor_path = 'shape_predictor_81_face_landmarks.dat'

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)

while(cap.isOpened()):
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    dets = detector(frame, 0)
    my_dict = {"69": [0,0], "73": [0,0], "74": [0,0],"24": [0,0],"19": [0,0],"75": [0,0]};
    nums = [69,73,74,24,19,75]
    for k, d in enumerate(dets):
        shape = predictor(frame, d)
        landmarks = np.matrix([[p.x, p.y] for p in shape.parts()])
        for num in range(shape.num_parts):
            #
            if str(num) in my_dict:
                my_dict[str(num)]=[shape.parts()[num].x,shape.parts()[num].y]
                cv2.circle(frame, (shape.parts()[num].x, shape.parts()[num].y), 3, (0,255,0), -1)
                z = str(shape.parts()[num].x) + ',' + str(shape.parts()[num].y)
                cv2.putText(frame, str(num), (shape.parts()[num].x, shape.parts()[num].y), cv2.FONT_HERSHEY_PLAIN,
                        1, (0, 255, 0), 1)

    centler = (math.ceil(frame.shape[1]/2)-1,math.ceil(frame.shape[0]/2)-1)
    print (center)
    print(my_dict)
    if (my_dict["75"][0] < center[0]) and (my_dict["69"][1] < center[1]) and (my_dict["19"][1] > center[1]) and (my_dict["74"][0] > center[0]):
        cv2.rectangle(frame, (math.ceil(frame.shape[1] / 2) - 40, math.ceil(frame.shape[0] / 2) - 40),
                        (math.ceil(frame.shape[1] / 2) + 40, math.ceil(frame.shape[0] / 2) + 40), (255, 0, 0), 3)
    cv2.rectangle(frame, (math.ceil(frame.shape[1]/2)-1,math.ceil(frame.shape[0]/2)-1), (math.ceil(frame.shape[1]/2)+1,math.ceil(frame.shape[0]/2)+1), (255,0,0), -1)
    cv2.imshow('frame', frame)
    out.write(frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("q pressed")
        break


cap.release()
out.release()

cv2.destroyAllWindows()




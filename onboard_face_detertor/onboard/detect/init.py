import numpy as np
import cv2
import os
import subprocess
import time 

face_cascade = cv2.CascadeClassifier('f.xml')

cap = cv2.VideoCapture(0)

while 1:
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    i = 0
    for (x,y,w,h) in faces:
        crop_img = img[y:y+h, x:x+w]
        file = '../faces/face_{}_{}_{}_{}.jpg'.format(x, y, w, h)
        cv2.imwrite(file, crop_img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        
        p = subprocess.Popen(['python', '../recog/one.py',  '{}'.format(file)]);
        p.wait()

        #cv2.imshow("face " + str(i), crop_img)
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        i += 1;

    cv2.imshow("img", img)

    #if i:
    #    time.sleep(2) 

    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()
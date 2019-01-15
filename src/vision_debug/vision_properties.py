import cv2
import json

cap = cv2.VideoCapture(0)
f = open('webcam_config.txt', 'w')

for prop in dir(cv2):
    if "CAP_PROP_" in prop:
        val = cap.get(getattr(cv2, prop))
        if not val == -1:
            print(prop, ' = ', val)
            f.write(prop + '=' + str(val)+'\n')

cap.release()
f.close()

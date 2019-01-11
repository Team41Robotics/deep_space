import cv2
import numpy as np

# Process for vision:
# 1) Get video feed
# 2) blur?
# 3) Apply grayscale
# 4) Threshold to black and white
# 5) Find contours

cap = cv2.VideoCapture(1)

# Load our webcam settings
f = open('webcam_config.txt', 'r')
settings = f.readlines()
f.close()
for line in settings:
    prop, val = line.split('=')
    cap.set(getattr(cv2, prop), float(val))

while True:

    ret, frame = cap.read()

    # Create black image
    blank = np.zeros((300, 300, 3), np.uint8)

    blur = cv2.GaussianBlur(frame, (9,9), 0)

    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray', gray)

    ret, thresh = cv2.threshold(gray, 127, 255, 0)
    #ret, thresh = cv2.threshold(gray, 230, 255, 0)
    cv2.imshow('thresh', thresh)

    im2, contours, hierarchy = cv2.findContours(thresh,
                                                cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('c'):
        cv2.imwrite('template.png', frame)
    
    if cv2.waitKey(1) == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()

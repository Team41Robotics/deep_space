import cv2
import numpy as np

# 1) Get video feed
# 2) blur?
# 3) Apply grayscale
# 4) Threshold to black and white
# 5) Find contours
# 6) Filter contours by:
#       - Width height ratio
#       - Minimum area
#       - Angle rotation
#
# 7)

DEGREE_TOLERANCE = 7
OPTIMAL_WH_RATIO = 4.0/11.0
WH_TOLERANCE = .30
AREA_TOLERANCE = 1000

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
    tapeFrame = frame

    # Blur image to reduce noise
    blur = cv2.GaussianBlur(frame, (9,9), 0)

    # Convert image to greyscale
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('gray', gray)

    # Convert image to black and white
    ret, thresh = cv2.threshold(gray, 175, 255, 0)
    cv2.imshow('thresh', thresh)

    # Find contours from black and white image
    im2, contours, hierarchy = cv2.findContours(thresh,
                                                cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)
#    best_right_tape = ((0,0),(0,0),0)
#    best_left_tape = ((0,0),(0,0),0)

    tapes_left = []
    tapes_right = []
    
    # Check every contour
    for contour in contours:

        # Draw bounding box around contour. This provides height, width, center,
        # and rotation information.
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        
        width = min(rect[1])
        height = max(rect[1])
        rotation = rect[2]

        # Check for width height ratio and 
        if(rect[1][1]!=0 and
           abs((width/height)-OPTIMAL_WH_RATIO)/(OPTIMAL_WH_RATIO)< WH_TOLERANCE and
           width*height > AREA_TOLERANCE):

            cv2.drawContours(tapeFrame, [box], 0, (0,0,255),2)

            # Check if bounding boxes have correct angle of rotation
            if abs(abs(rotation) - 75) < DEGREE_TOLERANCE:
                cv2.drawContours(tapeFrame, [box], 0, (255,255,0),2)
                for i in range(len(tapes_left)):
                    if (box < tape[i][0]):
                        tapes_left.insert(i, box)
                    elif i == len(tapes_left)-1:
                        tapes_left.append(box)
                
            if abs(abs(rotation) - 5) < DEGREE_TOLERANCE:
                cv2.drawContours(tapeFrame, [box], 0, (0,255,255),2)
                tapes_right.append(box)



    # Display frames            
    #cv2.imshow('rectangle', frame)
    cv2.imshow('tape detection', tapeFrame)

    # Take a pic and save to template.png
    if cv2.waitKey(1) == ord('c'):
        cv2.imwrite('template.png', frame)

    # Exit
    if cv2.waitKey(1) == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()

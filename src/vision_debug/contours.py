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
CAM_CENTER = (320,240)


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
    finalFrame = frame.copy()

    # Blur image to reduce noise
    blur = cv2.GaussianBlur(frame, (9,9), 0)

    # Convert image to greyscale
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('gray', gray)

    # Convert image to black and white
    ret, thresh = cv2.threshold(gray, 175, 255, 0)
    cv2.imshow('thresh', thresh)

    # Find contours from black and white image
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    tapes = []
    
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
        if(height!=0 and
           abs((width/height)-OPTIMAL_WH_RATIO)/(OPTIMAL_WH_RATIO)< WH_TOLERANCE and
           width*height > AREA_TOLERANCE):

            cv2.drawContours(tapeFrame, [box], 0, (0,0,255),2)

            # Check if bounding boxes have correct angle of rotation
            if abs(abs(rotation) - 75) < DEGREE_TOLERANCE:
                cv2.drawContours(tapeFrame, [box], 0, (255,255,0),2)
                largest = True
                for i in range(len(tapes)):
                    if (rect[0][0] < tapes[i][0][0][0]):
                        tapes.insert(i, [rect,1])
                        largest = False
                        break
                if largest:
                        tapes.append([rect,1])
                
            if abs(abs(rotation) - 5) < DEGREE_TOLERANCE:
                cv2.drawContours(tapeFrame, [box], 0, (0,255,255),2)
                largest = True
                for i in range(len(tapes)):
                    if (rect[0][0] < tapes[i][0][0][0]):
                        tapes.insert(i, [rect,0])
                        largest = False
                        break
                if largest:
                        tapes.append([rect,0])

    pairs = [] # array of pairs
    leftTape = True # start by looking for left tape
    for tape in tapes:
        if tape[1] == leftTape: # if tape matches the correct orientation 
            pairs.append(tape) # add it to the pairs
            leftTape = not leftTape # move on to the next tape orientation
        elif tape[1] == 1: # if tape is supposed to be a matching right but is left
            pairs.pop() # remove the left tape before it
            pairs.append(tape) #add the new left tape
    if  len(pairs) > 0 and pairs[-1][1] == 1:
        pairs.pop() # if the last tape is a left one remove it
        
        
    for tape in pairs:
        box = cv2.boxPoints(tape[0])
        box = np.int0(box)
        cv2.drawContours(finalFrame, [box], 0, (255,0,255),2)

    i = 0
    print(pairs)
    while i < len(pairs):
        center1 = pairs[i][0][0]
        center2 = pairs[i + 1][0][0]
        x = (center1[0]+center2[0])/2
        y = (center1[1]+center2[1])/2
        cv2.circle(finalFrame, (int(x),int(y)), 3, (255, 255, 255), -1)
        i += 2

    cv2.circle(finalFrame, CAM_CENTER, 3, (255, 255, 255), -1)
        
    # Display frames            
    #cv2.imshow('rectangle', frame)
    cv2.imshow('tape detection', tapeFrame)
    cv2.imshow('tape paired', finalFrame)

    # Take a pic and save to template.png
    if cv2.waitKey(1) == ord('c'):
        cv2.imwrite('template.png', frame)

    # Exit
    if cv2.waitKey(1) == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()

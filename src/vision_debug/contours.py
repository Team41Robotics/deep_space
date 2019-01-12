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
    tapeFrame = frame.copy()
    finalFrame = frame.copy()
    templateFrame = frame.copy()

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

        # Check for width height ratio and min area 
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

    leftTape = True
    print('length before' + str(len(tapes)))
    #print(tapes)

#    for i,tape in enumerate(tapes):
    if(len(tapes) % 2 == 1):
        #print(tapes[-1][1])
        if(tapes[len(tapes)-1][1]==1):
            print("Removed extra left tape")
            tapes.pop()
        elif tapes[0][1] != 0:
            print("Removed extra right tape")
            tapes.pop(0)

    i = 0
    while i < len(tapes):
        #print(tapes[i][1])
        print(i)
        if (tapes[i][1] != leftTape):
            print('removed tape:' + str(tapes[i][1]))
            tapes.pop(i)
            if not leftTape:
                tapes.pop(i-1)
        else:
            leftTape = not leftTape
            i += 1

    #print('done')
    #print(tapes)
    #print(len(tapes))

    if len(tapes) == 2:
        center1 = tapes[0][0][0]
        center2 = tapes[1][0][0]
        x = (center1[0]+center2[0])/2
        y = (center1[1]+center2[1])/2
        cv2.circle(finalFrame, (int(x),int(y)), 3, (255, 255, 255), -1) 
        cv2.circle(finalFrame, CAM_CENTER, 3, (255, 255, 255), -1)

    #for tape in tapes:
        
        
        #box = cv2.boxPoints(tape[0])
        #box = np.int0(box)
        #cv2.drawContours(finalFrame, [box], 0, (255,0,255),2)
    
    #i = 0
    #while i < len(tapes):
     #   if tape == leftTape:
      #      pairs[i][0] = tape
       # elif tape == rightTape:
        #    pairs[i][1] = tape
         #   i += 1
        
    # Display frames            
    #cv2.imshow('rectangle', frame)
    cv2.imshow('tape detection', tapeFrame)
    cv2.imshow('tape paired', finalFrame)
    cv2.imshow('template', templateFrame)

    # Take a pic and save to template.png
    if cv2.waitKey(1) == ord('c'):
        cv2.imwrite('template.png', templateFrame)

    # Exit
    if cv2.waitKey(1) == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()

import cv2
import numpy as np

# 1) Get video feed
# 2) blur?
# 3) Extract green channel
# 4) Threshold to black and white
# 5) Find contours
# 6) Filter contours by:
#       - Width height ratio
#       - Minimum area
#       - Angle rotation
#
# 7) Pair complementary tapes together
# 8) Find midpoints of centers of complementary tapes

DEGREE_TOLERANCE = 15
OPTIMAL_WH_RATIO = 4.0/11.0
WH_TOLERANCE = .50
AREA_TOLERANCE = 1000
#CAM_CENTER = (320,240)

LEFT_TAPE = 1
RIGHT_TAPE = 0

cap = cv2.VideoCapture(0)

cWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
cHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
CAM_CENTER = (int(cWidth/2), int(cHeight/2))

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
    cv2.imshow("frame", frame)

    # Blur image to reduce noise
    blur = cv2.GaussianBlur(frame, (9,9), 0)
    
    # Extract green channel
    g = blur[:,:,1]
    cv2.imshow('green', g)

    # Convert image to black and white
    ret, thresh = cv2.threshold(g, 205, 255, 0)
    cv2.imshow('thresh', thresh)

    # Find contours from black and white image
    img, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

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
           width*height >= AREA_TOLERANCE):

            cv2.drawContours(tapeFrame, [box], 0, (0,0,255),2)

            # Check if bounding boxes have correct angle of rotation
            if abs(abs(rotation) - 75) < DEGREE_TOLERANCE:
                cv2.drawContours(tapeFrame, [box], 0, (255,255,0),2)

                # Insert boxes from left to right
                largest = True
                for i in range(len(tapes)):
                    if (rect[0][0] < tapes[i][0][0][0]):
                        tapes.insert(i, [rect,1])
                        largest = False
                        break
                if largest:
                        tapes.append([rect,1])
            else:
                print("Failed left angle test: " + str(rotation))
                
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
            else:
                print("Failed right angle test: " + str(rotation))
        elif width*height < AREA_TOLERANCE:
            print("Area too small: " + str(width*height))
        else:
            print("WH Ratio off: " + str(abs((width/height)-OPTIMAL_WH_RATIO)/(OPTIMAL_WH_RATIO)))
                
    pairs = [] # array of pairs
    currTape = LEFT_TAPE # start by looking for left tape
    for tape in tapes:
        if tape[1] == currTape: # if tape matches the correct orientation 
            pairs.append(tape) # add it to the pairs
            currTape = not currTape # move on to the next tape orientation
        elif tape[1] == LEFT_TAPE: # if tape is supposed to be a matching right but is left
            pairs.pop() # remove the left tape before it
            pairs.append(tape) #add the new left tape

    if  len(pairs) > 0 and pairs[-1][1] == 1:
        pairs.pop() # if the last tape is a left one remove it
        
    # Draw boxes around pairs and points at their corners
    for tape in pairs:
        box = cv2.boxPoints(tape[0])
        box = np.int0(box)
        cv2.drawContours(finalFrame, [box], 0, (255,0,255),2)
        for i in range(4):
			cv2.circle(finalFrame, (int(box[i][0]),int(box[i][1])), 3, (0, 0, 0), -1)
			x = (box[i][0] / cWidth) * 2 - 1
			y = (box[i][1] / cHeight) * 2 - 1
			print("x " + str(x) + ", y " + str(y))

    # Draw midpoint between centers of each box in a pair
    i = 0
    while i < len(pairs):
        center1 = pairs[i][0][0]
        center2 = pairs[i + 1][0][0]
        x = (center1[0]+center2[0])/2
        y = (center1[1]+center2[1])/2
        cv2.circle(finalFrame, (int(x),int(y)), 3, (255, 255, 255), -1)
        print("Center x " + str((x / cWidth) * 2 - 1) + "\ny " + str((y / cHeight) * 2 - 1))
        i += 2

    # Draw dot in center of the screen
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

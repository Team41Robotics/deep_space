import cv2
import numpy as np

cap = cv2.VideoCapture(1)

lower = (0,104, 111)
upper = (255, 255 ,255)
kernel = np.ones((1,1),np.uint8)

def isolateOrange(src):
        lower = np.array([0, 103, 105])
        upper = np.array([150,255, 255])
        mask = cv2.inRange(src, lower, upper)
        output = cv2.bitwise_and(src, src, mask = mask)
        return output


while True:

    ret, frame = cap.read()

    # Blur image to reduce noise
    blur = cv2.GaussianBlur(frame, (21,21), 0)

    orange = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(orange, lower, upper)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations= 2)

    cv2.imshow("fdfd", mask)
    
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 150, param1=50, param2=24, minRadius = 30, maxRadius = 0)
    print(circles)

    if circles is not None: 
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            cv2.circle(frame, (i[0],i[1]),i[2],(255,0,0),2)

    #contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #cv2.drawContours(frame, contours, -1, (255,0,0), 2)
    cv2.imshow("frame", frame)
    
    # Exit
    if cv2.waitKey(1) == ord('q'):
	#my_file.close()
        break
    
cap.release()
cv2.destroyAllWindows()

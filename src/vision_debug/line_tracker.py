import cv2
import numpy as np

def isolateWhite(src):
    lower = np.array([100, 100, 100])
    upper = np.array([255,255, 255])
    mask = cv2.inRange(src, lower, upper)
    output = cv2.bitwise_and(src, src, mask = mask)
    return output

cap = cv2.VideoCapture(0)

mWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
mHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
CAM_CENTER = (int(mWidth/2), int(mHeight/2))

# Load our webcam settings

f = open('webcam_config.txt', 'r')
settings = f.readlines()
f.close()

#frame = cv2.imread("./field_img/CargoLine36in.jpg",1)
#mHeight,mWidth,channels = frame.shapeged
blur = cv2.GaussianBlur(frame, (11,11), 0)	

# Isolate white	


gray = isolateWhite(blur)
ret, thresh = cv2.threshold(gray, 180, 255, 0)
im_bw = cv2.cvtColor(thresh, cv2.COLOR_RGB2GRAY)
img, contours, hierarchy = cv2.findContours(im_bw,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(frame, contours, -1, (0,255,0), 2)

for contour in contours:
	#print(str(contour))
	# Draw bounding box around contour. This provides height, width, center,
	# and rotation information.
	rect = cv2.minAreaRect(contour)
	box = cv2.boxPoints(rect)
	box = np.int0(box)

	width = min(rect[1])
	height = max(rect[1])
	rotation = rect[2]

	cv2.drawContours(frame, [box], 0, (255,0,0),2)
	print("Rotation")
	print(str(rotation))
	
	# Draw boxes around pairs and points at their corners	
	'''
	box = cv2.boxPoints(rect)
	box = np.int0(box)
	cv2.drawContours(frame, [box], 0, (255,0,255),2)
	for i in range(4):
	   cv2.circle(frame, (int(box[i][0]),int(box[i][1])), 3, (0, 0, 0), -1)
	print("Width")
	print(box[i][0]/width)
	print("Height")
	print(box[i][1]/height)
	'''
	# Find Center
	M = cv2.moments(contour)
	cX = M["m10"] / M["m00"]
	cY = M["m01"] / M["m00"]
	print("Center X")
	print(str(cX/mWidth))
	print("Center Y")
	print(str(cY/mHeight))

cv2.imshow("tresh", im_bw)
cv2.imshow("picture", frame)

cv2.waitKey(0)
cv2.destroyAllWindows()

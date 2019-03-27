import cv2
import numpy as np
from distance_estimation import CameraEstimator
import os
from networktables import NetworkTables

NetworkTables.initialize(server='10.0.41.2')
sd = NetworkTables.getTable("SmartDashboard")

# 1) Get video feed
# 2) blur?
# 3) Extract green channel
# 4) Threshold to black and white
# 5) Find contours
# 6) Filter contours by:
#	   - Width height ratio
#	   - Minimum area
#	   - Angle rotation
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

cap = cv2.VideoCapture('/dev/video6')

cWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
cHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
CAM_CENTER = (int(cWidth/2), int(cHeight/2))

# Load our webcam settings
dir_path = os.path.dirname(os.path.abspath(__file__))

wconfig = open(dir_path+'/calibration_files/webcam_config.txt', 'r')
settings = wconfig.readlines()
wconfig.close()

for line in settings:
	prop, val = line.split('=')
	cap.set(getattr(cv2, prop), float(val))

ref_points_f = open(dir_path+'/calibration_files/reference_points.txt', 'r')
ref_points = []
for line in ref_points_f.readlines():
	x, y = line.split(',')
	ref_points.append([float(x),float(y)])
ref_points_f.close()

cal_data_f = open(dir_path+'/calibration_files/calibration.txt', 'r')
cal_data_str = cal_data_f.readline().split(',')
v_angle_x = float(cal_data_str[0])
v_angle_y = float(cal_data_str[1])
cal_dist = float(cal_data_str[2])
cal_span_x = float(cal_data_str[3])
cal_span_y = float(cal_data_str[4])
cal_data_f.close()

ce = CameraEstimator(ref_points,v_angle_x,v_angle_y,cal_dist,cal_span_x,cal_span_y)

def get_camera_data(debug_window):
	ret, frame = cap.read()
	tapeFrame = frame.copy()
	finalFrame = frame.copy()

  # Blur image to reduce noise
	blur = cv2.GaussianBlur(frame, (9,9), 0)
	
	# Extract green channel
	g = blur[:,:,1]

	# Convert image to black and white
	ret, thresh = cv2.threshold(g, 205, 255, 0)
	if debug_window:
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

		rotation = rect[2]

		if cv2.contourArea(contour) < (rect[1][0]*rect[1][1]) * .7:
			continue

		cv2.drawContours(tapeFrame, [box], 0, (0,0,255),2)

		# Check if bounding boxes have correct angle of rotation
		if abs(abs(rotation) - 75) < DEGREE_TOLERANCE:

			height = rect[1][0] # width is the longer side when tilted 75
			width = rect[1][1] # heigh is the shorter side when tilted 75

		 	# Check for width height ratio and min area 
			if(height!=0 and
			abs((width/height)-OPTIMAL_WH_RATIO)/(OPTIMAL_WH_RATIO)< WH_TOLERANCE and
			width*height >= AREA_TOLERANCE): 

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
				
		if abs(abs(rotation) - 15) < DEGREE_TOLERANCE:

			height = rect[1][1] # width is the longer side when tilted 75
			width = rect[1][0] # height is the shorter side when tilted 75

		 	# Check for width height ratio and min area 
			if(height!=0 and
			   abs((width/height)-OPTIMAL_WH_RATIO)/(OPTIMAL_WH_RATIO)< WH_TOLERANCE and
			   width*height >= AREA_TOLERANCE): 


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

	estimated_distance = None
	# Draw boxes around pairs and points at their corners
	for tape in pairs:
		box = cv2.boxPoints(tape[0])
		box = np.int0(box)
		cv2.drawContours(finalFrame, [box], 0, (255,0,255),2)
		for i in range(4):
			cv2.circle(finalFrame, (int(box[i][0]),int(box[i][1])), 3, (0, 0, 0), -1)
			x = (box[i][0] / cWidth) * 2 - 1
			y = (box[i][1] / cHeight) * 2 - 1

	# Draw midpoint between centers of each box in a pair
	i = 0
	area1 = 0
	while i < len(pairs):
		area2 = pairs[i][0][1][0]*pairs[i][0][1][1] + pairs[i+1][0][1][0]*pairs[i+1][0][1][1]		
		if area2 > area1:
			center1 = pairs[i][0][0]
			center2 = pairs[i + 1][0][0]
			x = (center1[0]+center2[0])/2
			y = (center1[1]+center2[1])/2
			cv2.circle(finalFrame, (int(x),int(y)), 3, (255, 255, 255), -1)
			box1 = cv2.boxPoints(pairs[i][0])
			box1 = np.int0(box1)

			box2 = cv2.boxPoints(pairs[i+1][0])
			box2 = np.int0(box2)
		
			box = np.concatenate((box1, box2))
			box_x = np.multiply(np.true_divide(box[:,0],cWidth),2)-1
			box_y = np.multiply(np.true_divide(box[:,1],cHeight),2)-1
			box = np.column_stack([box_x,box_y])
			estimated_distance = ce.get_distance(box)
			area1 = area2
			sd.putNumber("horizontal_offset_normalized", (x-CAM_CENTER[0])/(CAM_CENTER[0]*2))
		i += 2

	# Draw dot in center of the screen
	cv2.circle(finalFrame, CAM_CENTER, 3, (255, 255, 255), -1)
		
	# Display frames			
	#cv2.imshow('rectangle', frame)
	if debug_window:
		cv2.imshow('tape detection', tapeFrame)
		cv2.imshow('tape paired', finalFrame)

	return estimated_distance

def get_camera_interrupt(): 
	if cv2.waitKey(1) == ord('q'):
		cap.release()
		cv2.destroyAllWindows()
		print(destroyed)
		return True
	return False

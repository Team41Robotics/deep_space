import numpy as np


#read from raw data and get a filtered set

f = open('captured_reference_image.txt', 'r')
raw_data = f.readlines()
f.close()

data = []

for line in raw_data:
	if(line[0] == 'x'):		
		data.append(line)

f.close()

#sort each corner (x, y) into an array
corners = []
for i in range(8): # 8 corners
	corners.append([[], []]) # [x coords, y coords]

for i in range(len(data)):
	corner_num = i % 8
	points = data[i].split(" ")
	x = float(points[1][:-1])
	y = float(points[3])
	corners[corner_num][0].append(x)
	corners[corner_num][1].append(y)

for corner in corners:
	x_avg = sum(corner[0]) / len(corner[0])
	y_avg = sum(corner[1]) / len(corner[1])
	print("x " + str(x_avg) + ", y " + str(y_avg))

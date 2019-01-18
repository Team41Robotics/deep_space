import numpy as np
import math

class CameraEstimator: 
	# it is assumed that x and y are already normalized
	
	def __init__(self, ref, _view_angle_x, _view_angle_y, _cal_dist, _cal_span_x, _cal_span_y): 
		ref = np.matrix(ref)
		ref_x = ref[:,0]
		ref_y = ref[:,1]
		self.view_angle_x = math.radians(_view_angle_x)
		self.view_angle_y = math.radians(_view_angle_y)
		self.cal_dist = _cal_dist
		self.cal_span_x = np.matrix(_cal_span_x)
		self.cal_span_y = np.matrix(_cal_span_y)
		self.Ax = np.matrix(np.column_stack([ref_x,-np.ones(len(ref_x))]))
		self.Ay = np.matrix(np.column_stack([ref_y,-np.ones(len(ref_y))]))
		# calculate the least mean squared coefficients given system Ax=b
		# where b is observed points normalized and x is sigma
		# calculating the coefficient A beforehand so it need not be calculated each iteration
		self.Alsx = (self.Ax.T*self.Ax).I*self.Ax.T	
		self.Alsy = (self.Ay.T*self.Ay).I*self.Ay.T	

		# using one matrix for x and y:
		#self.A = np.matrix((Ax,Ay))
		#self.Alms = (self.A.T*self.A).I*self.A.T

	def get_distance(self, observed_points):
		observed_points = np.matrix(observed_points)
		soltn_x = self.Alsx*observed_points[:,0]
		soltn_y = self.Alsy*observed_points[:,1]
		sigma_dx = np.asscalar(soltn_x[0])
		sigma_dy = np.asscalar(soltn_y[0])
		tau_x = np.asscalar(soltn_x[1]/sigma_dx)
		tau_y = np.asscalar(soltn_y[1]/sigma_dy)
		ttx = math.tan(self.view_angle_x/2)
		tty = math.tan(self.view_angle_y/2)
		dist_est_x = self.cal_dist + self.cal_span_x / (2 * ttx) * (1 - sigma_dx) / sigma_dx
		dist_est_y = self.cal_dist + self.cal_span_y / (2 * tty) * (1 - sigma_dy) / sigma_dy

		dist = np.asscalar((dist_est_x+dist_est_y)/2)
		x = np.asscalar(self.cal_span_x/2*tau_x)
		y = np.asscalar(self.cal_span_y/2*tau_y)

		return [dist,dist_est_x,dist_est_y,x,y]

#estimator = CameraEstimator([[.1,.4],[.2,.5],[.3,.6]], 55, 77, 2, 4,10)
#print(estimator.get_distance([[.2,.4],[.3,.5],[.4,.6]]))

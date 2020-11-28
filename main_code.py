# -*- coding: utf-8 -*-
from robot_EKF import RobotEKF
from numpy.linalg import norm
from math import cos, acos
from math import sin
from math import acos


def get_rotation_angle(rot):
	return acos(rot[0][0])

def get_rotation_matrix(theta):
	rot = np.array([[cos(theta), -sin(theta), 0],[sin(theta), cos(theta), 0],[0,0,1]])
	return rot

def track_orientation(prev, ang_vel,dt):
	ang_vel = ang_vel.flatten()

	#B is the cross product matrix form of the angular velocity vector
	B = np.array([[0, -ang_vel[2], ang_vel[1]], [ang_vel[2], 0, -ang_vel[0]], [-ang_vel[1], ang_vel[0], 0]]) * dt
	sigma = norm(dt)*norm(ang_vel)
	
	if sigma == 0:
		update_factor = 1
	else:
		update_factor = np.eye(3, dtype=float) + sin(sigma)*B/sigma + (1-cos(sigma))*np.matmul(B,B)/(sigma**2)

	rot = np.dot(prev,update_factor)
	return rot

def track_position(rot, acc, vel, pos, dt):
	#Gravidade compensada junto do bias
	#g = np.array([[0,0,-9.8]]).T
	g = np.array([[0.,0.,0.]]).T

	#TODO encontrar melhor modelo aqui
	acc = np.matmul(rot, acc)
	pos = pos + dt*vel + acc*dt**2/2
	vel = vel + dt*(acc-g)
	vel = vel + dt*(acc-g)
	pos = pos + dt*vel 
	return acc, vel, pos


std_range=0.3
std_bearing=0.1

ekf = RobotEKF(dt=1.0)


#Initialize filter parameters
ekf.x = np.array([[0,0,0,0,0,0]])
ekf.P = np.diag([.5,.5,.5,.5,.5,.5]])
ekf.R = np.diag([std_range**2, std_bearing**2])

ekf.calibration(calibration_time=30)




step = 10

#while True:
for i in range(200):	
	time.sleep(1)
	ekf.predict()

	#ekf.update()
	detected_landmarks = read_landmarks()
	if len(detected_landmarks):
		for lmark in detected_landmarks:
				lmark_id = lmark[0][0]
				z = np.array([[ lmark[0][1], lmark[0][2] ]])

				lmark_real_pos = np.array()		 ##TODO	

                ekf.update(z, HJacobian=self.h_jacobian(lmark_real_pos), Hx=self.h(lmark_real_pos), residual=self.residual, args=(lmark), hx_args=(lmark))

	





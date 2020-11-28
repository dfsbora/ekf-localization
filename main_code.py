# -*- coding: utf-8 -*-
from robot_EKF import RobotEKF
from numpy.linalg import norm
from math import cos, acos
from math import sin
from math import acos
import time
import numpy as np

DEBUG = 1


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


#########################################################

std_range=0.3
std_bearing=0.1

gt_landmarks =  {}
gt_landmarks[85] = np.array([[0,0]])



ekf = RobotEKF(dt=1.0)

if DEBUG:
	print "RobotEKF Object created"


#Initialize filter parameters
ekf.x = np.array([[0,0,0,0,0,0]]).T
#ekf.P = np.diag([[.5,.5,.5,.5,.5,.5]])
ekf.P *= 0.5
print ekf.P
ekf.R = np.diag([std_range**2, std_bearing**2])

if DEBUG:
	print "Parameters initialized"


ekf.calibration(calibration_time=2)

if DEBUG:
	print "IMU calibrated"


step = 20
#while True:
for i in range(120):	
	time.sleep(0.5)
	ekf.predict()

	#ekf.update()
	detected_landmarks = ekf.read_landmarks()

	if detected_landmarks is None:
		ekf.update(z=detected_landmarks)
		continue

	for lmark in detected_landmarks:
		lmark_id = lmark[0]
		if DEBUG:
			print "Detected " + lmark_id

		z = np.array([[ lmark[1], lmark[2] ]])
		lmark_real_pos = gt_landmarks.get(lmark_id) #check lmark_id type
			print lmark_real_pos

        #ekf.update(z, HJacobian=ekf.h_jacobian, Hx=ekf.h, residual=ekf.residual, args=(lmark_real_pos), hx_args=(lmark_real_pos))
        ekf.update(z, lmark_real_pos)

	if i % step == 0 and DEBUG:
		print '***************'
		print 'i =  %d' % (i)
		print ekf.x

	





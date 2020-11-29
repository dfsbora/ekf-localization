# -*- coding: utf-8 -*-
from robot_EKF import RobotEKF
from numpy.linalg import norm
from math import cos, acos
from math import sin
from math import acos
import time
import numpy as np


# Debug variables
DEBUG = 1
DEBUG_DETAIL = 0
PLOT = 0
PRINT_STEP = 60

# Execution variables
N = 120
TIME_SLEEP = 0.5
CALIBRATION_TIME = 10


#########################################################
# NOT USED FUNCTIONS

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

# MAP
# Create a field map Dictionary as "NAOmark ID: (x,y)" in global positions
field_map =  {}
field_map[85] = np.array([[0,0]]).T


# Create instance of robot filter
ekf = RobotEKF(dt=TIME_SLEEP)

if DEBUG:
	print "RobotEKF Object created"


# FILTER INITIALIZATION
# Variances
std_range = 0.5
std_bearing = 0.5

# Filter parameters initialization
# State
ekf.x = np.array([[2.0,0,0,0,0,0]]).T
# Uncertainty covariance
ekf.P *= 0.5
# Process Uncertainty
ekf.Q *= 0.5
# Measurement uncertainty
ekf.R = np.diag([std_range**2, std_bearing**2])


if DEBUG:
	print "Parameters initialized"


# CALIBRATION
# Calculate gyroscope and accelerometer bias
ekf.calibration(calibration_time=CALIBRATION_TIME)

if DEBUG:
	print "IMU calibrated"
	print "\n\n"

x_prior_array = []
p_prior_array = []
x_post_array = []
p_post_array = []


time.sleep(TIME_SLEEP)


#move_id = ekf.move()

# LOCALIZATION LOOP

#i=0
for i in range(N):	
#while ekf.motion_proxy.moveIsActive():
	# PREDICT
	acc, gyro = ekf.read_sensors()
	ekf.compensate_bias(acc,gyro)

	u = np.array([[ekf.acc[0][0], ekf.acc[1][0], ekf.gyro[2][0]]]).T
	ekf.predict(u,dt=0.5)


	if (i % PRINT_STEP) == 0 and DEBUG:
		print("***************")
		print("Prediction")
		print(ekf.x)
		print('\n')

	if PLOT:
		x_prior_array.append(ekf.x)
		p_prior_array.append(ekf.P)
	
	# UPDATE
	detected_landmarks = ekf.read_landmarks()

	# No feature detected
	if detected_landmarks is None:
		# Update step copies predicted values
		ekf.update(z=detected_landmarks)
		continue

	# Update filter for each detected feature
	for lmark in detected_landmarks:
		lmark_id = lmark[0]

		if DEBUG_DETAIL:
			print("Detected landmark number: ", lmark_id)

		# Create measurement array as [distance, angle]
		z = np.array([[ lmark[1], lmark[2] ]]).T

		# Get landmark gt position
		lmark_real_pos = field_map.get(lmark_id) 

        ekf.update(z, lmark_real_pos)

	if (i % PRINT_STEP) == 0 and DEBUG:
		print("Update")
		print(ekf.x)
		print('\n')

	if PLOT:
		x_post_array.append(ekf.x)
		p_post_array.append(ekf.P)

	time.sleep(TIME_SLEEP)
	i += 1

print("Final position: ", ekf.x)
print("Final P: ", ekf.P[0][0])


if PLOT:
	import csv
	with open("output.csv", "wb") as f:
		writer = csv.writer(f)
		writer.writerows(x_prior_array)


# Robot goes to rest position
#time.sleep(TIME_SLEEP)
ekf.motion_proxy.waitUntilMoveIsFinished()
ekf.motion_proxy.rest()
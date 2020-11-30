# -*- coding: utf-8 -*-
from robot_EKF import RobotEKF
from numpy.linalg import norm
from math import cos, acos
from math import sin
from math import acos
import time
import numpy as np
import qi
import unboard

session = qi.Session()

def initialize():
	ekf = RobotEKF(dt=0.5, session=session)

def main():
	# Debug variables
	DEBUG = 1
	DEBUG_DETAIL = 0
	PLOT = 0
	PRINT_STEP = 50

	# Execution variables
	N = 120
	TIME_SLEEP = 0.5
	CALIBRATION_TIME = 30

	# MAP
	# Create a field map Dictionary as "NAOmark ID: (x,y)" in global positions
	field_map =  {}
	field_map[85] = np.array([[0,0]]).T

	# Create instance of robot filter
	ekf = RobotEKF(dt=0.5, session=session)
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


	unboard.is_calibrated = True


	# LOCALIZATION LOOP
	i=0
	while True:	
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

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
import logging


session = qi.Session()

def initialize():
	ekf = RobotEKF(dt=0.5, session=session)

def main():
	# Debug variables
	PLOT = 0
	PRINT_STEP = 50

	# Execution variables
	N = 120
	TIME_SLEEP = 0.5
	CALIBRATION_TIME = 30

	# MAP
	# Create a field map Dictionary as "NAOmark ID: (x,y)" in global positions
	field_map =  { 85: np.array([[0,0]]).T ,  64: np.array([[0,25]]).T}
	#field_map[85] = 

	# Create instance of robot filter
	logging.info("Creating RobotEKF object...")
	ekf = RobotEKF(dt=0.5, session=session)


	# FILTER INITIALIZATION
	# Variances
	logging.info("Initializing filter parameters...")
	std_range = 0.1
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


	# CALIBRATION
	# Calculate gyroscope and accelerometer bias
	ekf.calibration(calibration_time=CALIBRATION_TIME)


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


		if (i % PRINT_STEP) == 0:
			logging.debug("**********")
			logging.debug("Prediction")
			logging.debug(ekf.x)
			logging.debug("**********")
	

		if PLOT:
			x_prior_array.append(ekf.x)
			p_prior_array.append(ekf.P)
		
		
		# UPDATE
		

		# No feature detected
		if unboard.got_landmark is False:
			# Update step copies predicted values
			logging.debug("Didnt find landmark")
			ekf.update(z=None)
			continue

		detected_landmarks = unboard.landmarks
		# Update filter for each detected feature
		for lmark in detected_landmarks:
			lmark_id = lmark[0][0]

			# Create measurement array as [x, y]
			z = np.array([[ lmark[1], lmark[2] ]]).T

			# Get landmark gt position
			lmark_real_pos = field_map.get(lmark_id) 

	        ekf.update(z, lmark_real_pos)


		if (i % PRINT_STEP) == 0:
			logging.debug("**********")
			logging.debug("Update")
			logging.debug(ekf.x)
			logging.debug("**********")

		if PLOT:
			x_post_array.append(ekf.x)
			p_post_array.append(ekf.P)

		time.sleep(TIME_SLEEP)
		i += 1

	logging.info("Final position: %s", ekf.x)
	logging.info("Final P: %s", ekf.P[0][0])


	if PLOT:
		import csv
		with open("output.csv", "wb") as f:
			writer = csv.writer(f)
			writer.writerows(x_prior_array)

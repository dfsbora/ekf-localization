# -*- coding: utf-8 -*-
from robot_EKF import RobotEKF
from numpy.linalg import norm
from math import cos, acos
from math import sin, asin
import time
import numpy as np
import qi
import unboard
import logging


session = qi.Session()



def main():
	# Debug and auxiliar variables
	PLOT = 0
	PRINT_STEP = 10
	TIME_SLEEP = 0.5
	CALIBRATION_TIME = 60

	x_prior_array = []
	p_prior_array = []
	x_post_array = []
	p_post_array = []

	# MAP
	# Create a field map Dictionary as "NAOmark ID: (x,y)" in global positions
	#field_map =  { 85: np.array([[0, 4.0]]).T ,  64: np.array([[1.5, 1.]]).T}
	field_map =  { 85: np.array([[0.5, 2.0]]).T ,  64: np.array([[1.5, 1.]]).T}


	# Create instance of robot filter
	logging.info("Creating RobotEKF object...")
	ekf = RobotEKF(dt=0.5, session=session)

	# FILTER INITIALIZATION
	# Variances
	std_x = 0.1
	std_y = 0.5

	# Filter parameters initialization
	# State
	ekf.x = np.array([[0,0,0,0,0,0,1,0,0,0,1,0,0,0,1]]).T
	# Uncertainty covariance
	ekf.P *= 0.5
	# Process Uncertainty
	ekf.Q *= 0.5
	# Measurement uncertainty
	ekf.R = np.diag([std_x**2, std_y**2])

	# CALIBRATION
	# Calculate gyroscope and accelerometer bias
	ekf.calibration(calibration_time=CALIBRATION_TIME)

	unboard.is_calibrated = True



	# LOCALIZATION LOOP
	i=0
	prev_time = time.time()
	while True:	

		# PREDICT
		time.sleep(.2)
		current_time = time.time()
		acc, gyro = ekf.read_sensors()
		ekf.compensate_bias(acc,gyro)

		u = np.concatenate((ekf.acc,ekf.gyro),axis=0)
		
		dt = current_time-prev_time
		ekf.predict(u=u,dt=dt)
		prev_time=current_time
		ekf.angle_from_rotation_matrix()

		if (i%PRINT_STEP)==0:
			logging.debug("Prediction")
			logging.debug("x: %s", ekf.x[0][0])
			logging.debug("y: %s", ekf.x[2][0])
			#logging.debug("rot: %s, %s, %s", ekf.x[6][0], ekf.x[10][0], ekf.x[14][0])	
			logging.debug("angle: %s", ekf.angle)


		if PLOT:
			x_prior_array.append(ekf.x)
			p_prior_array.append(ekf.P)
		
		
		# UPDATE

		# No feature detected
		if unboard.got_landmark is False:
			# Update step copies predicted values
			#logging.debug("Didnt find landmark")
			ekf.update(z=None)
			continue

		detected_landmarks = unboard.landmarks
		# Update filter for each detected feature
		try:
			for lmark in detected_landmarks:
				lmark_id = lmark[0][0]

				# Create measurement array as [x, y]
				z = np.array([[ lmark[1], lmark[2] ]]).T

				# Get landmark gt position
				lmark_real_pos = field_map.get(lmark_id) 

				ekf.update(z, lmark_real_pos)
				ekf.angle_from_rotation_matrix()

			if (i%PRINT_STEP)==0:
				logging.debug("Update")
				logging.debug("x: %s", ekf.x[0][0])
				logging.debug("y: %s", ekf.x[2][0])
				logging.debug("rot: %s, %s, %s", ekf.x[6][0], ekf.x[10][0], ekf.x[14][0])
				logging.debug("angle: %s", ekf.angle)

			if PLOT:
				x_post_array.append(ekf.x)
				p_post_array.append(ekf.P)
		except:
			pass

		#time.sleep(TIME_SLEEP)
		i = i + 1

	logging.info("Final position: %s", ekf.x)
	logging.info("Final P: %s", ekf.P[0][0])


	if PLOT:
		import csv
		with open("output.csv", "wb") as f:
			writer = csv.writer(f)
			writer.writerows(x_prior_array)

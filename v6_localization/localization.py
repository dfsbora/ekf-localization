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
	# Save data in file aux variables
	PLOT = 1
	PLOT_STEP = 10
	FILENAME = "output.txt"
	# Data arrays 
	x_prior_array = []
	p_prior_array = []
	x_post_array = []
	p_post_array = []
	dt_array = []


	# Logging aux variables
	i=0
	PRINT_STEP = 350
	TIME_SLEEP = 0
	CALIBRATION_TIME = 120


	# MAP
	# Create a field map Dictionary as "NAOmark ID: (x,y)" in global positions
	field_map =  { 64: np.array([[3.0, 0.]]).T, 85: np.array([[3.0, 2.25]]).T , 108: np.array([[2.05, 2.25]]).T}	


	logging.info("Creating RobotEKF object...")
	ekf = RobotEKF(dt=0.5, session=session)

	# PARAMETERS INITIALIZATION
	# State
	ekf.x = np.array([[0,0,0.0,0,0,0,1,0,0,0,1,0,0,0,1]]).T
	# Uncertainty covariance
	ekf.P *= np.diag([0.2,0.,0.2,0.,0,0., 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
	# Process Uncertainty
	ekf.Q *= np.diag([0.25**2,0.25,0.25**2,0.25,0.25**2,0.25, 0.0001, 0., 0., 0., 0.0001, 0., 0., 0., 0.0001])
	# Measurement uncertainty
	ekf.R = np.diag([0.0825**2, 0.0825**2])


	# CALIBRATION
	ekf.calibration(calibration_time=CALIBRATION_TIME)
	unboard.is_calibrated = True


	# LOCALIZATION LOOP

	prev_time = time.time()
	plot_prev_time = prev_time
	
	while unboard.run_localization :

		##############
		# PREDICT STEP

		acc, gyro = ekf.read_sensors()
		ekf.compensate_bias(acc,gyro)
		u = np.concatenate((ekf.acc,ekf.gyro),axis=0)		

		current_time = time.time()
		dt = current_time-prev_time
		prev_time = current_time

		ekf.predict(u=u,dt=dt)

		if (i%PRINT_STEP)==0:
			logging.debug("Prediction")
			logging.debug("x: %s", ekf.x[0][0])
			logging.debug("y: %s", ekf.x[2][0])
			logging.debug("P: %s", ekf.P[0][0])
			logging.debug("P: %s", ekf.P[2][2])
			ekf.angle_from_rotation_matrix()
			logging.debug("angle: %s", np.degrees(ekf.angle))

		if PLOT and (i%PLOT_STEP)==0:
			plot_current_time = current_time
			plot_dt = plot_current_time - plot_prev_time
			plot_prev_time = plot_current_time

			dt_array.append(plot_dt)
			x_prior_array.append(ekf.x)
			p_prior_array.append(ekf.P)
		

		#############
		# UPDATE STEP

		# No feature detected, copies predicted values
		if unboard.got_landmark is False:
			ekf.update(z=None)
			
		else:
			detected_landmarks = unboard.landmarks
			try:
				# Update filter for each detected feature
				for lmark in detected_landmarks:
					lmark_id = lmark[0][0][0]

					# Create measurement array as [x, y]
					z = np.array([[ lmark[1][0], lmark[2][0] ]]).T
					# Get landmark gt position
					lmark_real_pos = field_map.get(lmark_id) 

					ekf.update(z, lmark_real_pos)
					
			except Exception:
				pass

		if (i%PRINT_STEP)==0:
			logging.debug("Update")
			logging.debug("Saw landmark: %s", unboard.got_landmark)
			logging.debug("x: %s", ekf.x[0][0])
			logging.debug("y: %s", ekf.x[2][0])
			logging.debug("P: %s", ekf.P[0][0])
			logging.debug("P: %s", ekf.P[2][2])
			ekf.angle_from_rotation_matrix()
			logging.debug("angle: %s", np.degrees(ekf.angle))

		if PLOT and (i%PLOT_STEP)==0:
			x_post_array.append(ekf.x)
			p_post_array.append(ekf.P)

		i = i + 1
		

	# OUTPUT FILE
	# Write output file with x and P in predict and update steps
	if PLOT:
		with open(FILENAME,"w") as f:
			for x_prior, p_prior, x_post, p_post, dt in zip(x_prior_array, p_prior_array, x_post_array, p_post_array, dt_array):
				f.write(str(dt))
				f.write(";")
				for x in x_prior:
					f.write(str(x[0])+",")
				f.write(";")
				for row in p_prior:
					for p in row:
						f.write(str(p)+",")
				f.write(";")

				for x in x_post:
					f.write(str(x[0])+",")
				f.write(";")
				for row in p_post:
					for p in row:
						f.write(str(p)+",")
				f.write("\n")


	# FINAL POSITION
	logging.info("Final position: %s", ekf.x)
	logging.info("\n")
	logging.info("Final P: %s", ekf.P)
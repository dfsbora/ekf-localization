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
	PRINT_STEP = 30
	TIME_SLEEP = 0.2
	CALIBRATION_TIME = 21

	x_prior_array = []
	p_prior_array = []
	x_post_array = []
	p_post_array = []


	# MAP
	# Create a field map Dictionary as "NAOmark ID: (x,y)" in global positions
	field_map =  { 85: np.array([[0, 4.0]]).T ,  64: np.array([[1.5, 1.]]).T}
	#field_map =  { 85: np.array([[0, 4.0]]).T ,  64: np.array([[0.5, 0]]).T}
	

	# Create instance of robot filter
	logging.info("Creating RobotEKF object...")
	ekf = RobotEKF(dt=0.5, session=session)

	# Filter parameters initialization
	# State
	ekf.x = np.array([[0,0,1.,0,0,0,1,0,0,0,1,0,0,0,1]]).T
	# Uncertainty covariance
	ekf.P *= 0.5
	# Process Uncertainty
	#ekf.Q *= 
	# ekf.Q = np.zeros((15, 15))
	# ekf.Q[1][1] = std_x
	# ekf.Q[3][3] = std_y
	# ekf.Q[5][5] = std_z

	# Measurement uncertainty
	ekf.R = np.diag([0.1**2, 0.1**2])


	# CALIBRATION
	# Calculate gyroscope and accelerometer bias
	ekf.calibration(calibration_time=CALIBRATION_TIME)
	unboard.is_calibrated = True


	# LOCALIZATION LOOP
	i=0
	prev_time = time.time()
	while True:	

		# PREDICT
		
		acc, gyro = ekf.read_sensors()
		ekf.compensate_bias(acc,gyro)
		u = np.concatenate((ekf.acc,ekf.gyro),axis=0)		

		current_time = time.time()
		dt = current_time-prev_time
		prev_time=current_time

		ekf.predict(u=u,dt=dt)
		ekf.angle_from_rotation_matrix()

		if (i%PRINT_STEP)==0:
			logging.debug("Prediction")
			logging.debug("x: %s", ekf.x[0][0])
			logging.debug("y: %s", ekf.x[2][0])
			logging.debug("P: %s", ekf.P[0][0])
			logging.debug("angle: %s", np.degrees(ekf.angle))

		if PLOT:
			x_prior_array.append(ekf.x)
			p_prior_array.append(ekf.P)
		
		

		# UPDATE

		# No feature detected
		if unboard.got_landmark is False:
			# Update step copies predicted values
			#logging.debug("didnt")
			ekf.update(z=None)
			continue

		detected_landmarks = unboard.landmarks
		try:
			# Update filter for each detected feature
			for lmark in detected_landmarks:
				lmark_id = lmark[0][0][0]
				#logging.debug(lmark_id)

				# Create measurement array as [x, y]
				z = np.array([[ lmark[1][0], lmark[2][0] ]]).T
				#logging.debug(z)

				#print("z: ", z)
				# Get landmark gt position
				lmark_real_pos = field_map.get(lmark_id) 
				#logging.debug(lmark_real_pos)

				ekf.update(z, lmark_real_pos)
				logging.debug("update")
				ekf.angle_from_rotation_matrix()
				#logging.debug("angle")


			#logging.debug(i%PRINT_STEP)
			if (i%PRINT_STEP)==0:
				logging.debug("z: %s", z)
				logging.debug("real: %s", lmark_real_pos)
				logging.debug("Update")
				logging.debug("x: %s", ekf.x[0][0])
				logging.debug("y: %s", ekf.x[2][0])
				logging.debug("P: %s", ekf.P[0][0])
				logging.debug("angle: %s", np.degrees(ekf.angle))

			if PLOT:
				x_post_array.append(ekf.x)
				p_post_array.append(ekf.P)


		except:
			pass



		i = i + 1
		time.sleep(TIME_SLEEP)


	logging.info("Final position: %s", ekf.x)
	logging.info("Final P: %s", ekf.P[0][0])


	if PLOT:
		import csv
		with open("output.csv", "wb") as f:
			writer = csv.writer(f)
			writer.writerows(x_prior_array)


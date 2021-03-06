# -*- coding: utf-8 -*-

"""Copyright 2020 Débora Ferreira dos Santos

Adapted from Roger R Labbe Jr. FilterPy library.
http://github.com/rlabbe/filterpy

This is licensed under an MIT license.
"""

from extended_kalman_filter import ExtendedKalmanFilter as EKF
from numpy import array, sqrt, dot
from math import sin, cos, tan, atan, asin, acos, atan2
import numpy as np
from numpy.random import rand
import scipy.linalg as linalg
from copy import deepcopy
import time
import logging
from numpy.linalg import norm

class RobotEKF(EKF):
    def __init__(self, dt,session):
        
        EKF.__init__(self, 15, 2)

        # INITIALIZE SERVICES
        try:
            self.mem_service = session.service("ALMemory")  
            self.motion_service = session.service("ALMotion")
        except Exception, e:
            logging.error("Error when creating services: %s", e)
            exit(1)

        # IMU
        # Create gyroscope and accelerometer atributes
        self.acc_bias = np.zeros((3, 1))
        self.gyro_bias = np.zeros((3, 1))
        self.angle = 0

        self.acc = np.zeros((3, 1))        
        self.gyro = np.zeros((3, 1))
        self.g = np.array([[0.,0.,9.81]]).T

    def angle_from_rotation_matrix(self):
        """ Get angle around z from rotation matrix
        """
        pitch1 = -asin(self.x[12][0])
        pitch2 = np.pi - asin(self.x[12][0])
        yaw1 = atan2(self.x[9][0]/cos(pitch1), self.x[6][0]/cos(pitch1))
        yaw2 = atan2(self.x[9][0]/cos(pitch2), self.x[6][0]/cos(pitch2))
        self.angle = yaw1

    def calibration(self,calibration_time=120):
        """ Perform IMU sensors calibration

        Parameters
        ----------
        calibration_time : int
            Duration of calibration
        """
        logging.info("Starting calibration...")

        self.motion_service.setStiffnesses("Body", 1.0)
        self.motion_service.moveInit()

        acc_sum = np.array([[0., 0., 0.]]).T
        gyro_sum = np.array([[0., 0., 0.]]).T

        i = 0
        initial_time =  time.time()

        # Auxiliar time variable used to log calibration status
        counter_prev = time.time()

        while (time.time() - initial_time) < calibration_time:
            acc, gyro = self.read_sensors()
            acc_sum = acc_sum + acc
            gyro_sum = gyro_sum + gyro
            i = i+1

            if (time.time() - counter_prev) > 20:
                logging.info("Calibration in progress...")
                counter_prev=time.time()

        acc_bias = acc_sum/i + np.array([[0.,0.,9.81]]).T
        gyro_bias = gyro_sum/i

        self.acc_bias = acc_bias 
        self.gyro_bias = gyro_bias

        logging.debug("Acc bias: %s", acc_bias)
        logging.debug("Gyro bias: %s", gyro_bias)

    def read_sensors(self):
        """ Read IMU sensors data
        """
        accX = self.mem_service.getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value")
        accY = self.mem_service.getData("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value")
        accZ = self.mem_service.getData("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value")

        gyroX = self.mem_service.getData("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value")
        gyroY = self.mem_service.getData("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value")
        gyroZ = self.mem_service.getData("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value")

        acc = np.array([[accX,accY,accZ]]).T  
        gyro = np.array([[gyroX,gyroY,gyroZ]]).T 

        return acc, gyro
        
    def compensate_bias(self, acc, gyro):
        """ Compensate bias on IMU reading

        Parameters
        ----------
        acc : np.array
            accelerometer reading

        gyro : np.array
            gyroscope reading
        """

        prev = np.array([[self.x[6][0], self.x[7][0], self.x[8][0]],
            [self.x[9][0], self.x[10][0], self.x[11][0]],
            [self.x[12][0], self.x[13][0], self.x[14][0]]])

        #Project bias to current orientation
        acc_bias = np.dot(prev, self.acc_bias)

        self.acc = acc - acc_bias  
        self.gyro = gyro - self.gyro_bias

    def h(self, lmark):
        """ Transform state to measurement space
        Parameters
        ----------
        lmark : 

        Returns
        ----------
        hx : 
            state transformed to measurement space
        """
        dx = lmark[0][0] - self.x[0][0]
        dy = lmark[1][0] - self.x[2][0]        

        coordinates = np.array([[dx,dy]]).T
        rotation = np.array([[self.x[6][0],self.x[7][0]],[self.x[9][0],self.x[10][0]]])

        hx = dot(rotation,coordinates)
        return hx

    def h_jacobian(self, lmark):
        """ Compute jacobian (linearized) measurement matrix H

        Parameters
        ----------
        lmark : 

        Returns
        ----------
        H : 
            jacobian of h
            
        """

        # Alternative Jacobian calculation. Do not correspond to actual system model.
        H = np.zeros((2, 15))
        H[0][0] = self.x[6][0]
        H[0][2] = self.x[7][0]
        H[1][0] = self.x[9][0]
        H[1][2] = self.x[10][0]
        H =  -H

        return H

    def residual(self, a, b):
        """ Calculate residual

        Parameters
        ----------
        a : 
            measurement
        b :
            state transformed do measurement

        Returns
        ----------
        y : 
            residual
        """

        #Note that if working with angles, range should be forced to be in range [0, 2 pi)
        y = (a - b)
        return y

    def update(self, z, lmark_real_pos=None, R=None):
        """ Performs the update innovation of the extended Kalman filter.

        Parameters
        ----------

        z : np.array
            measurement for this step.
            If `None`, posterior is not computed

        HJacobian : function
           function which computes the Jacobian of the H matrix (measurement
           function). Takes state variable (self.x) as input, returns H.

        Hx : function
            function which takes as input the state variable (self.x) along
            with the optional arguments in hx_args, and returns the measurement
            that would correspond to that state.

        R : np.array, scalar, or None
            Optionally provide R to override the measurement noise for this
            one call, otherwise  self.R will be used.

        args : tuple, optional, default (,)
            arguments to be passed into HJacobian after the required state
            variable. for robot localization you might need to pass in
            information about the map and time of day, so you might have
            `args=(map_data, time)`, where the signature of HCacobian will
            be `def HJacobian(x, map, t)`

        hx_args : tuple, optional, default (,)
            arguments to be passed into Hx function after the required state
            variable.

        residual : function (z, z2), optional
            Optional function that computes the residual (difference) between
            the two measurement vectors. If you do not provide this, then the
            built in minus operator will be used. You will normally want to use
            the built in unless your residual computation is nonlinear (for
            example, if they are angles)
        """

        
        #No measurement
        if z is None:
            self.z = np.array([[None]*self.dim_z]).T
            self.x_post = self.x.copy()
            self.P_post = self.P.copy()
            return

        #Keep internal R the same, otherwise update it
        if R is None:
            R = self.R
        elif np.isscalar(R):
            R = eye(self.dim_z) * R

        #Assure it is a vector
        if np.isscalar(z) and self.dim_z == 1:
            z = np.asarray([z], float)

        #Define h_jacobian to be used in calculations
        H = self.h_jacobian(lmark_real_pos)
         
        #Calculate Kalman Gain
        PHT = dot(self.P, H.T)

        self.S = dot(H, PHT) + R
        self.SI = linalg.inv(self.S)
        self.K = PHT.dot(self.SI)

        #Calculate residual using defined residual function
        hx = self.h(lmark_real_pos)
        self.y = self.residual(z, hx)
        self.x = self.x + dot(self.K, self.y)

        # P = (I-KH)P(I-KH)' + KRK' is more numerically stable
        # and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature.
        # Update P
        I_KH = self._I - dot(self.K, H)
        self.P = dot(I_KH, self.P).dot(I_KH.T) + dot(self.K, R).dot(self.K.T)

        # save measurement and posterior state
        self.z = deepcopy(z)
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def f(self, u, dt):
        """ Calculates x from previous x and current u

        Parameters
        ----------
        u : 
            
        """

        prev = np.array([[self.x[6][0], self.x[7][0], self.x[8][0]],
            [self.x[9][0], self.x[10][0], self.x[11][0]],
            [self.x[12][0], self.x[13][0], self.x[14][0]]])

        pos = np.array([[self.x[0][0],self.x[2][0],self.x[4][0]]]).T
        vel = np.array([[self.x[1][0],self.x[3][0],self.x[5][0]]]).T

        acc = np.array([[u[0][0],u[1][0],u[2][0]]]).T
        
        ang_vel = np.array([[u[3][0],u[4][0],u[5][0]]]).T

        sigma = norm(dt)*norm(ang_vel)
        
        if sigma == 0:
            update_factor = 1
        else:
            B = np.array([[0, -ang_vel[2][0], ang_vel[1][0]], [ang_vel[2][0], 0, -ang_vel[0][0]], [-ang_vel[1][0], ang_vel[0][0], 0]]) * dt
            update_factor = np.eye(3, dtype=float) + sin(sigma)*B/sigma + (1-cos(sigma))*np.matmul(B,B)/(sigma**2)

        rot = np.dot(prev,update_factor)

        acc = np.dot(prev,acc)
        acc = acc + self.g
        pos = pos + dt*vel
        vel = vel + dt*acc

        self.x = np.array([[pos[0][0], vel[0][0], pos[1][0], vel[1][0], pos[2][0], vel[2][0], rot[0][0], rot[0][1], rot[0][2], rot[1][0], rot[1][1], rot[1][2], rot[2][0], rot[2][1], rot[2][2]]]).T

    def f_jacobian(self, dt):
        """ Compute jacobian F (df/dx)

        Parameters
        ----------
        dt : int

        """

        # Alternative Jacobian calculation. Do not correspond to actual system model.    
        F = np.eye(self.dim_x)
        F[0][1] = dt
        F[2][3] = dt
        F[4][5] = dt

        return F

    def v_jacobian(self, dt, prev):
        """ Compute jacobian V (df/du)

        Parameters
        ----------
        dt : int

        prev: np.array
            previous rotation matrix

        """    

        # Alternative Jacobian calculation. Do not correspond to actual system model.    
        V = np.zeros((15,6))
        V[1,0:3] = prev[0]*dt
        V[3,0:3] = prev[1]*dt
        V[5,0:3] = prev[2]*dt

        return V

    def predict(self, u,dt):
        self.f(u, dt)

        prev = np.array([[self.x[6][0], self.x[7][0], self.x[8][0]],
            [self.x[9][0], self.x[10][0], self.x[11][0]],
            [self.x[12][0], self.x[13][0], self.x[14][0]]])

        F = self.f_jacobian(dt)
        FPFT = dot(F,self.P).dot(F.T)

        self.P = FPFT  + self.Q 

        # Note that if considering the noise in control, previous line should be something like:
        # V = self.v_jacobian(dt, prev)
        # covariance of motion noise in control space
        # M = np.diag([0.0265**2, 0.01567**2, 0.0253**2, 0.0015**2, 0.0031**2, 0.0019**2])
        # VMVT = dot(V,M).dot(V.T)
        # self.P = FPFT  + VMVT 

# -*- coding: utf-8 -*-
from extended_kalman_filter import ExtendedKalmanFilter as EKF
from numpy import array, sqrt, dot
from math import sin, cos, tan, atan
import numpy as np
from numpy.random import rand
import scipy.linalg as linalg
from copy import deepcopy
import time
import logging

class RobotEKF(EKF):
    def __init__(self, dt,session):
        
        EKF.__init__(self, 6, 2)
        self.F = np.array(
            [[1, dt, 0, 0, 0, 0],
             [0,1,0,0,0,0],
             [0,0,1,dt,0,0],
             [0,0,0,1,0,0],
             [0,0,0,0,1,dt],
             [0,0,0,0,0,1]])

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

        self.acc = np.zeros((3, 1))        
        self.gyro = np.zeros((3, 1))


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

        # Auxiliar time variable to log calibration status
        counter_prev = time.time()

        while (time.time() - initial_time) < calibration_time:
            acc, gyro = self.read_sensors()
            acc_sum = acc_sum + acc
            gyro_sum = gyro_sum + gyro
            i = i+1

            if (time.time() - counter_prev) > 30:
                logging.info("Calibration in progress...")
                counter_prev=time.time()

        acc_bias = acc_sum/i
        gyro_bias = gyro_sum/i

        self.acc_bias = acc_bias #TODO retirar gravidade daqui
        self.gyro_bias = gyro_bias

        logging.debug("Acc bias: %s", acc_bias)
        logging.debug("Gyro bias: %s", gyro_bias)


    def get_vel_pos(self):
        vel = np.array([[ ekf.x[1], ekf.x[3], 0]])
        pos = np.array([[ ekf.x[0], ekf.x[2], 0]])
        return vel, pos


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
        self.acc = acc - self.acc_bias  #TODO precisa compensar a rotação aqui?
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

        dx = self.x[0][0] - lmark[0][0]
        dy = self.x[2][0] - lmark[1][0]
        theta = self.x[4][0]
        coordinates = np.array([[dx,dy]]).T
        
        rotation = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])

        hx = dot(rotation,coordinates)

        #logging.debug("x in measurement space: %s", hx)


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
        theta = self.x[4][0]
        H = np.array([[cos(theta),0,-sin(theta),0,0,0],[sin(theta),0,cos(theta),0,0,0]])

        #logging.debug("H jacobian: %s", H)

        return H


    def residual(self, a, b):
        """ Calculate residual and force the angle to be in range [0, 2 pi)

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

        y = (a - b)

        #logging.debug("Residual: %s", y)
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

        #logging.debug("Kalman gain: %s", self.K)

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
        # ax = u[0][0]
        # ay = u[1][0]
        # wz = u[2][0]
        # theta = self.x[4]
        ax = 0
        ay = 0
        wz = 0
        theta = 0

        self.x[0] = self.x[0] + self.x[1]*dt + 0.5*(ax*cos(theta) - ay*sin(theta))*dt**2
        self.x[1] = self.x[1] + (ax*cos(theta) - ay*sin(theta))*dt
        self.x[2] = self.x[2] + self.x[3]*dt + 0.5*(ax*sin(theta) + ay*cos(theta))*dt**2
        self.x[3] = self.x[3] + (ax*sin(theta) + ay*cos(theta))*dt
        self.x[4] = self.x[4] + wz*dt
        self.x[5] = wz




    def f_jacobian(self, dt):
        """ Compute jacobian F (df/dx)

        Parameters
        ----------
        dt : int

        """    
        F = np.array(
            [[1, dt, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0],
             [0, 0, 1, dt, 0, 0],
             [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 0]])

        return F

    def v_jacobian(self, theta, dt):
        """ Compute jacobian V (df/du)

        Parameters
        ----------
        dt : int

        """    
        V = np.array(
            [[cos(theta)*0.5*dt**2, -sin(theta)*0.5*dt**2, 0],
             [cos(theta)*dt, -sin(theta)*dt, 0],
             [sin(theta)*0.5*dt**2, cos(theta)*0.5*dt**2, 0],
             [sin(theta)*dt, cos(theta)*dt, 0],
             [0, 0, 0],
             [0, 0, 1]])

        return V

    def predict(self, u,dt):
        theta = self.x[4]
        self.f(u, dt)

        F = self.f_jacobian(dt)
        V = self.v_jacobian(theta, dt)

        # covariance of motion noise in control space
        #TODO ajustar M
        # M = array([[0.5**2, 0, 0], 
        #            [0, 0.5**2, 0],
        #            [0, 0, 0.7**2]])
        M = array([[0, 0, 0], 
                   [0, 0, 0],
                   [0, 0, 0]])
        VMVT = dot(V,M).dot(V.T)
        FPFT = dot(F,self.P).dot(F.T)
        self.P = FPFT + VMVT
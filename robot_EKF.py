# -*- coding: utf-8 -*-
from extended_kalman_filter import ExtendedKalmanFilter as EKF
from numpy import array, sqrt, dot
from math import sin, cos, tan, atan
import numpy as np
from numpy.random import rand
import scipy.linalg as linalg
from copy import deepcopy
import time
from naoqi import ALProxy


DEBUG = 1

class RobotEKF(EKF):
    def __init__(self, dt):
        EKF.__init__(self, 6, 2)  #(self, dim_x, dim_z,  dim_u=0)
        self.dt = dt # predict period

        self.F = np.array(
            [[1, dt, 0, 0, 0, 0],
             [0,1,0,0,0,0],
             [0,0,1,dt,0,0],
             [0,0,0,1,0,0],
             [0,0,0,0,1,dt],
             [0,0,0,0,0,1]])

        #Initialize proxies
        robotIP = "nao.local"
        PORT = 9559
        fpsTime = 1/20

        try: 
            self.motion_proxy  = ALProxy("ALMotion", robotIP, PORT)
            self.mem_proxy = ALProxy("ALMemory","localhost", PORT)
            self.lmark_proxy = ALProxy("ALLandMarkDetection", robotIP, PORT)
            self.mem_value = "LandmarkDetected"

        except Exception, e:
            print "Error when creating proxies"
            print str(e)
            exit(1)


        self.gyro_bias = np.zeros((3, 1))
        self.acc_bias = np.zeros((3, 1))
        
        self.gyro = np.zeros((3, 1))
        self.acc = np.zeros((3, 1))
    

    def move(self, x, dt):
        change = np.array(
            [[1, dt, 0, 0, 0, 0],
             [0,1,0,0,0,0],
             [0,0,1,dt,0,0],
             [0,0,0,1,0,0],
             [0,0,0,0,1,dt],
             [0,0,0,0,0,1]])

        dx = dot(change, x) + rand(6,1)*0.2  #generates data

        return x + dx

    #TODO
    def calibration(self,calibration_time=120):
        self.motion_proxy.setStiffnesses("Body", 1.0)
        self.motion_proxy.moveInit()

        acc_sum = np.array([[0., 0., 0.]]).T
        gyro_sum = np.array([[0., 0., 0.]]).T
        initial_time =  time.time()
        counter_prev = time.time()
        i = 0

        while (time.time() - initial_time) < calibration_time:
            acc, gyro = self.read_sensors()
            acc_sum = acc_sum + acc
            gyro_sum = gyro_sum + gyro
            i = i+1

            if (time.time() - counter_prev) > 30:
                if DEBUG:
                    print("Calibration in progress")
                    print("Acc reading:", acc)
                    print("Gyro reading:", gyro)
                counter_prev=time.time()

        acc_bias = acc_sum/i
        gyro_bias = gyro_sum/i

        self.acc_bias = acc_bias
        self.gyro_bias = gyro_bias

        if DEBUG:
            print("Acc bias: ", acc_bias)
            print("Gyro bias: ", gyro_bias)


    def get_vel_pos(self):
        vel = np.array([[ ekf.x[1], ekf.x[3], 0]])
        pos = np.array([[ ekf.x[0], ekf.x[2], 0]])
        return vel, pos


    def read_sensors(self):
        accX = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value")
        accY = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value")
        accZ = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value")

        gyroX = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value")
        gyroY = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value")
        gyroZ = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value")

        acc = np.array([[accX,accY,accZ]]).T 
        gyro = np.array([[gyroX,gyroY,gyroZ]]).T 
        
        return acc, gyro 

    def compensate_bias(self, acc, gyro):
        self.acc = acc - self.acc_bias  #TODO precisa compensar a rotação aqui?
        self.gyro = gyro - self.gyro_bias

    # Read memory for detected landmarks. Return ID, distance, angle.
    def read_landmarks(self):
        val = self.mem_proxy.getData(self.mem_value, 0)

        #landmarks = []
        landmarks = None

        if(val and isinstance(val, list) and len(val) >= 2):
            #time_stamp = val[0]
            landmarks = []
            mark_info_array = val[1]
            camera_pose = val[2]
        
            try:
                # Get info on each detected mark.
                for mark_info in mark_info_array:

                    mark_id = mark_info[1][0] #number of landmark
                    #Distance regression
                    distance = 0.134*mark_info[0][3]**(-1.04) #width
                    angle = np.degrees(mark_info[0][1]) #alpha

                    landmark = [mark_id, distance, angle]
                    landmarks.append(landmark)

                    # if DEBUG:
                    #     print "distance %.2f" % (distance)
                    #     print "angle %.1f" % (angle)

            except Exception:
                pass

        return landmarks


    def h(self, lmark):
        dx = self.x[0][0] - lmark[0][0]
        dy = self.x[2][0] - lmark[1][0]
        distance = sqrt(dx**2+dy**2)
        landmark_angle = atan(dy/dx)

        #if alpha < 0:
        #    angle = self.x[4] - landmark_angle
        #else:
        angle = landmark_angle - self.x[4]   #Labbe faz assim apenas

        H = np.array([[distance, angle]])

        if DEBUG:
            print "\n"
            print("**********")
            print("H")
            print(self.x[0])
            print(lmark[0])
            print("dx: ", dx)
            print("dy: ", dy)
            print("angle: ", angle)
            print("distance: ", distance)
            print("H: ", H)
            print "\n"


        return H

    #Linearizado
    def h_jacobian(self, lmark):
        dx = self.x[0][0] - lmark[0][0]
        dy = self.x[2][0] - lmark[1][0]
        hyp = dx**2+dy**2
        distance = sqrt(hyp)

        H = np.array([[dx/distance , 0, dy/distance, 0, 0, 0],
            [dy/hyp , 0, dx/hyp, 0, -1, 0]])

        # if DEBUG:
        #     print "\n"
        #     print("**********")
        #     print("H Jacobian")
        #     print(self.x[0][0])
        #     print(lmark[0][0])
        #     print("dx: ", dx)
        #     print("dy: ", dy)
        #     print("hyp: ", hyp)
        #     print("distance: ", distance)
        #     print("H: ", H)
        #     print "\n"

        return H

    def residual(self, a, b):
        y = (a - b).T


        if DEBUG:
            print "\n"
            print("**********")
            print(a)
            print(b)
            print(y)
            print "\n"

        y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
        if y[1] > np.pi:             # move to [-pi, pi)
            y[1] -= 2 * np.pi
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

        #Define HJacobian to be used in calculations
        #H = HJacobian(self.x, *args)
        H = self.h_jacobian(lmark_real_pos)

        #Calculate Kalman Gain
        PHT = dot(self.P, H.T)
        if DEBUG:
            print("H: ", H)
            print("P: ", self.P)
            print("PHT: ", PHT)
            print("R: ", R)
            print "\n\n"      

        self.S = dot(H, PHT) + R
        #TODO working here
        # if DEBUG:
        #     print("S: ", self.S)
        #     print("Type of S: ", type(self.S))
        #     print("Elements type: ", self.S.dtype)
        #     print "\n\n"
        self.SI = linalg.inv(self.S)
        self.K = PHT.dot(self.SI)

        #Calculate residual using defined residual function
        #hx = Hx(self.x, *hx_args)
        hx = self.h(lmark_real_pos)
        self.y = self.residual(z, hx)
        self.x = self.x + dot(self.K, self.y)

        # P = (I-KH)P(I-KH)' + KRK' is more numerically stable
        # and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature.
        #Update P
        I_KH = self._I - dot(self.K, H)
        self.P = dot(I_KH, self.P).dot(I_KH.T) + dot(self.K, R).dot(self.K.T)

        # save measurement and posterior state
        self.z = deepcopy(z)
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

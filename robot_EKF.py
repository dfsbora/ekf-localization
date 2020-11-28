# -*- coding: utf-8 -*-
from extended_kalman_filter import ExtendedKalmanFilter as EKF
from numpy import array, sqrt, dot
from numpy import sin, cos, tan
import numpy as np
from numpy.random import rand

import time
from naoqi import ALProxy


DEBUG = 0

class RobotEKF(EKF):
    def __init__(self, dt):
        EKF.__init__(self, 6, 3)  #(self, dim_x, dim_z,  dim_u=0)
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
            self.mem_proxy = ALProxy("ALMemory","localhost", PORR)
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
                print("acc:", acc)
                print("gyro:", gyro)
                counter_prev=time.time()

        self.acc_bias = acc_sum/i
        self.gyro_bias = gyro_sum/i


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

        landmarks = []
        if(val and isinstance(val, list) and len(val) >= 2):
            #time_stamp = val[0]
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

                    if DEBUG:
                        print "distance %.2f" % (distance)
                        print "angle %.1f" % (angle)

            except Exception:
                pass

        return landmarks


    def h(self, lmark):
        dx = self.x[0] - lmark[0]
        dy = self.x[2] - lmark[1]
        distance = sqrt(dx**2+dy**2)
        landmark_angle = atan(dy/dx)


        #if alpha < 0:
        #    angle = self.x[4] - landmark_angle
        #else:
        angle = landmark_angle - self.x[4]   #Labbe faz assim apenas

        H = np.array([[distance, angle]])
        return H


    #Linearizado
    def h_jacobian(self, lmark):
        dx = self.x[0] - lmark[0]
        dy = self.x[2] - lmark[1]
        hyp = dx**2+dy**2
        distance = sqrt(hyp)

        H = np.array([[dx/distance , 0, dy/distance, 0, 0, 0],
            [dy/hyp , 0, dx/hyp, 0, -1, 0]])


    def residual(a, b):
        y = a - b
        y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
        if y[1] > np.pi:             # move to [-pi, pi)
            y[1] -= 2 * np.pi
        return y

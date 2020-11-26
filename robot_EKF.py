from extended_kalman_filter import ExtendedKalmanFilter as EKF
from numpy import array, sqrt, dot
from numpy import sin, cos, tan
import numpy as np
from numpy.random import rand

import time
from naoqi import ALProxy

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

        self.motion_proxy  = ALProxy("ALMotion", robotIP, PORT)
        self.mem_proxy = ALProxy("ALMemory","localhost",9559)

        self.gyro_bias = 0.
        self.acc_bias = 0.
        
        self.gyro = 0.
        self.acc = 0
    

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


from EKF2 import ExtendedKalmanFilter as EKF
from numpy import array, sqrt, dot
from numpy import sin, cos, tan
import numpy as np
from sympy.abc import alpha, x, y, v, w, R, theta
from sympy import symbols, Matrix, evalf
import sympy
from numpy.random import rand

import time
from naoqi import ALProxy

class RobotEKF(EKF):
    def __init__(self, dt):
        EKF.__init__(self, 6, 3)  #(self, dim_x, dim_z,  dim_u=0)
        self.dt = dt # predict period

        #x, y, theta, x_vel, y_vel, theta_vel, time = symbols('x, y, theta, x_vel, y_vel, theta_vel')

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
        
        self.gyro_bias = 0.
        self.acc = 0
    




        # self.fxu = Matrix(
        #     [[time*x_vel],
        #      [0],
        #      [time*y_vel],
        #      [0],
        #      [time*theta_vel],
        #      [0]])

        #self.F_j = self.fxu.jacobian(Matrix([x, y, theta]))
        #self.V_j = self.fxu.jacobian(Matrix([v, a]))

        # save dictionary and it's variables for later use
        #self.subs = {x_vel: 0, y_vel: 0, time:0.,  theta_vel:0}
        #self.x_x, self.x_y, = x, y 
        #self.theta =  theta

    # def predict(self, u):
    #     self.x = self.move(self.x, u, self.dt)

    #     # self.subs[self.x_vel] = self.x[1, 0]
    #     # self.subs[self.y_vel] = self.x[3,0]
    #     # self.subs[self.theta_vel] = self.x[5,0]
    #     # self.subs[self.time] = self.dt

    #     self.F = np.array(
    #         [[1, dt, 0, 0, 0, 0],
    #          [0,1,0,0,0,0],
    #          [0,0,1,dt,0,0],
    #          [0,0,0,1,0,0],
    #          [0,0,0,0,1,dt],
    #          [0,0,0,0,0,1]])

    #     #F = array(self.F_j.evalf(subs=self.subs)).astype(float)
    #     #V = array(self.V_j.evalf(subs=self.subs)).astype(float)

    #     # covariance of motion noise in control space
    #     #M = array([[self.std_vel*u[0]**2, 0], 
    #     #           [0, self.std_steer**2]])

    #     #VMVT = dot(V,M).dot(V.T)
    #     FPFT = dot(F,self.P).dot(F.T)
    #     self.P = FPFT + self.Q
    #     #self.P = FPFT + VMVT


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
            acc, gyro = read_sensors(mem_proxy)
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
        gyroX = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value")
        gyroY = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value" )
        gyroZ = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value" )

        accX = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value")
        accY = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value")
        accZ = self.mem_proxy.getData("Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value")

        self.gyro = np.array([[gyroX,gyroY,gyroZ]]).T - self.gyro_bias
        self.acc = np.array([[accX,accY,accZ]]).T - self.acc_bias

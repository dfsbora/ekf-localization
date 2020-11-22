from EKF2 import ExtendedKalmanFilter as EKF
from numpy import array, sqrt, dot
from numpy import sin, cos, tan
import numpy as np
from sympy.abc import alpha, x, y, v, w, R, theta
from sympy import symbols, Matrix, evalf
import sympy

class RobotEKF(EKF):
    #UPDATE corrigir inicializacao 
    def __init__(self, dt):
        EKF.__init__(self, 6, 3, 0.1)  #(self, dim_x, dim_z, dt, dim_u=0)
        self.dt = dt # predict period

        #x, y, theta, x_vel, y_vel, theta_vel, time = symbols('x, y, theta, x_vel, y_vel, theta_vel')

        self.F = np.array(
            [[1, dt, 0, 0, 0, 0],
             [0,1,0,0,0,0],
             [0,0,1,dt,0,0],
             [0,0,0,1,0,0],
             [0,0,0,0,1,dt],
             [0,0,0,0,0,1]])

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

    #UPDATE move
    def move(self, x, u, dt):
        hdg = x[2, 0]
        vel = u[0]
        steering_angle = u[1]
        dist = vel * dt

        if abs(steering_angle) > 0.001: # is robot turning?
            beta = (dist / self.wheelbase) * tan(steering_angle)
            r = self.wheelbase / tan(steering_angle) # radius

            dx = np.array([[-r*sin(hdg) + r*sin(hdg + beta)], 
                           [r*cos(hdg) - r*cos(hdg + beta)], 
                           [beta]])
        else: # moving in straight line
            dx = np.array([[dist*cos(hdg)], 
                           [dist*sin(hdg)], 
                           [0]])
        return x + dx
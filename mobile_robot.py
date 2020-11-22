#import sympy
#from sympy.abc import alpha, x,y,v,w,R, theta
#from sympy import symbols, Matrix
from numpy.random import randn
from math import sin, cos, tan, sqrt, atan2
import matplotlib.pyplot as plt
import numpy as np
from robotEKF2 import RobotEKF
from plot_results import plot_ellipse

def residual(a, b):
    y = a - b
    y[1] = y[1] % (2 * np.pi)    # force in range [0, 2 pi)
    if y[1] > np.pi:             # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y

#UPDATE H_OX JACOBIANA DA DE BAIXO
def H_of(x, landmark_pos):
    px = landmark_pos[0]
    py = landmark_pos[1]
    hyp = (px - x[0, 0])**2 + (py - x[1, 0])**2
    dist = sqrt(hyp)

    H = np.array(
        [[-(px - x[0, 0]) / dist, -(py - x[1, 0]) / dist, 0],
         [ (py - x[1, 0]) / hyp,  -(px - x[0, 0]) / hyp, -1]])
    return H

#UPDATE HX  FUNCAO DE MEDIDA
def Hx(x, landmark_pos):
    px = landmark_pos[0]
    py = landmark_pos[1]
    dist = sqrt((px - x[0, 0])**2 + (py - x[1, 0])**2)
    Hx = np.array([[dist], [atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
    return Hx


#UPDATE MEDIDAS , AQUI ENTRAM OS DADOS DA CAMERA
def z_landmark(lmark, sim_pos, std_rng, std_brg):
    x, y = sim_pos[0, 0], sim_pos[1, 0]
    d = np.sqrt((lmark[0] - x)**2 + (lmark[1] - y)**2)  
    a = atan2(lmark[1] - y, lmark[0] - x) - sim_pos[2, 0]
    z = np.array([[d + randn()*std_rng],
                  [a + randn()*std_brg]])
    return z


dt = 1.0


#UPDATE: VALORES RECEBIDOS PELA FUNCAO
def run_localization(landmarks, std_vel, std_steer, 
                     std_range, std_bearing,
                     step=10, ellipse_step=20, ylim=None):
    # UPDATE VALORES PASSADOS PARA FUNCAO E REDEFINIR MATRIZES
    ekf = RobotEKF(dt, wheelbase=0.5, std_vel=std_vel, 
                   std_steer=std_steer)
    ekf.x = np.array([[2, 6, .3]]).T # x, y, steer angle
    ekf.P = np.diag([.1, .1, .1])
    ekf.R = np.diag([std_range**2, std_bearing**2])

    sim_pos = ekf.x.copy() # simulated position
    # steering command (vel, steering angle radians)
    u = np.array([1.1, .01]) 

    plt.figure()
    plt.scatter(landmarks[:, 0], landmarks[:, 1],
                marker='s', s=60)
    
    track = []
    for i in range(200):
        sim_pos = ekf.move(sim_pos, u, dt/10.) # simulate robot
        track.append(sim_pos)

        if i % step == 0:
            ekf.predict(u=u)

            if i % ellipse_step == 0:
            	ax = plt.gca()
            	plot_ellipse(ax, (ekf.x[0,0],ekf.x[2,0]), ekf.P[0:3,0:3], xIndex=0, yIndex=2, facecolor='k', alpha=0.3)

            x, y = sim_pos[0, 0], sim_pos[1, 0]
            for lmark in landmarks:
                z = z_landmark(lmark, sim_pos, std_range, std_bearing)
                ekf.update(z, HJacobian=H_of, Hx=Hx, residual=residual, args=(lmark), hx_args=(lmark))

            if i % ellipse_step == 0:
            	plot_ellipse(ax, (ekf.x[0,0],ekf.x[2,0]), ekf.P[0:3,0:3], xIndex=0, yIndex=2)
    track = np.array(track)
    plt.plot(track[:, 0], track[:,1], color='k', lw=2)
    plt.axis('equal')
    plt.title("EKF Robot localization")
    if ylim is not None: plt.ylim(*ylim)
    plt.show()
    return ekf

#UPDATE LANDMARKS
landmarks = np.array([[5, 10], [10, 5], [15, 15]])

#UPDATE CORRIGIR ARGUMENTOS
ekf = run_localization(landmarks, std_vel=0.1, std_steer=np.radians(1),std_range=0.3, std_bearing=0.1)

print('Final P:', ekf.P.diagonal())
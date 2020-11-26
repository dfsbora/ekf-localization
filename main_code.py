# -*- coding: utf-8 -*-
from robot_EKF import RobotEKF

ekf = RobotEKF(dt=1.0)

acc, gyro =  ekf.read_sensors()
print acc


while True:
	ekf.predict()


	print(ekf.x)





# if i % step == 0:
#     ekf.predict(u=u)

#     if i % ellipse_step == 0:
#         ax = plt.gca()
#         plot_ellipse(ax, (ekf.x[0,0],ekf.x[2,0]), ekf.P[0:3,0:3], xIndex=0, yIndex=2, facecolor='k', alpha=0.3)

#     x, y = sim_pos[0, 0], sim_pos[1, 0]
#     for lmark in landmarks:
#         z = z_landmark(lmark, sim_pos, std_range, std_bearing)
        

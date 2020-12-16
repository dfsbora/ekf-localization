import time
import qi
from naoqi import ALProxy
import unboard

session = qi.Session()


# moveTo(x,y,theta) 
# for velocity command use moveToward()
# moveToward(vx,vy,omega)

# Enable movements used for filter tests
zigzag = 0
square = 0
straight_line = 0
rectangle = 0
turn_around = 0
move_L = 1



def main():

	motion_service = session.service("ALMotion")
	motion_service.setStiffnesses("Body", 1.0)

	while unboard.run_localization:
		if unboard.is_calibrated:
			if zigzag:
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,1.57)
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,1.57)
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,-1.57)
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,-1.57)			
				unboard.run_localization = False

			elif square:
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,-1.57)
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,-1.57)
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,-1.57)
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,-1.57)
				unboard.run_localization = False

			elif straight_line:
				motion_service.moveTo(1.5, 0, 0)
				unboard.run_localization = False

			elif turn_around:
				motion_service.moveTo(0, 0,  3.1415)
				motion_service.moveTo(0, 0,  3.1415)
				unboard.run_localization = False

			elif move_L:
				motion_service.moveTo(2.5, 0,  0)
				motion_service.moveTo(0, 0, 1.5708)
				motion_service.moveTo(1.75, 0, 0)
				unboard.run_localization = False

			else:
				#no movement
				time.sleep(65)
				unboard.run_localization = False

			



import time
import qi
from naoqi import ALProxy
import unboard

session = qi.Session()


zigzag = 0
square = 0

def main():

	motion_service = session.service("ALMotion")
	motion_service.setStiffnesses("Body", 1.0)

	while unboard.run_localization:
		if unboard.is_calibrated:
			#motion_service.moveInit()
			if zigzag:
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,1.57)
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,1.57)
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,-1.57)
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,-1.57)

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
				motion_service.moveTo(1.0, 0, 0)
				motion_service.moveTo(0,0,-1.57)
				motion_service.moveTo(0.5, 0, 0)
				motion_service.moveTo(0,0,-1.57)
				motion_service.moveTo(1.0, 0, 0)
				motion_service.moveTo(0,0,-1.57)
				unboard.run_localization = False
			else:
				time.sleep(65)
				unboard.run_localization = False
			#motion_service.moveToward(0.3, 0, 0)
			#motion_service.moveTo(0, 0.5, 0)
			time.sleep(1)


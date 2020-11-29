import time
import qi
from naoqi import ALProxy


session = qi.Session()


def main():
	motion_service = session.service("ALMotion")

	# Wake up robot
	#motion_service.wakeUp()
	motion_service.setStiffnesses("Body", 1.0)
	motion_service.moveInit()
	motion_service.moveToward(1., 0, 0)


	# try:
	#	 motion_service.moveToward(1., 0, 0)
	# except Exception, message:
	#	 logging.error("Fail walk" + str(message))
	#	 exit()



def main2():

	robotIP = "nao.local"
	PORT = 9559

	motion_proxy  = ALProxy("ALMotion", robotIP, PORT)
	motion_proxy.post.moveTo(1.0,0,0)
	

	


	#motion_proxy.post.moveTo(1.0,0,0, _async=True)
	#move_id = self.motion_proxy.moveTo(1.0,0,0)#, _async=True)
	#return move_id

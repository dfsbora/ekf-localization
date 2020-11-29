import time
import qi
from naoqi import ALProxy


session = qi.Session()


def main():
	motion_service = session.service("ALMotion")

	motion_service.setStiffnesses("Body", 1.0)
	motion_service.moveInit()
	motion_service.moveToward(1., 0, 0)


#! /usr/bin/env python
# -*- encoding: UTF-8 -*-


import qi
import time
import sys
import argparse
import logging
import modulo


session = qi.Session()
def main():

    modulo.run(session)

    # mem_service = session.service("ALMemory")
    # lmark_service = session.service("ALLandMarkDetection")

    # try:
    #     while True:
    #         val = mem_service.getData("LandmarkDetected", 0)
    #         print("val ", val)
    #         time.sleep(0.5)
    #         print("\n")
    # except KeyboardInterrupt:
    #     print "Interrupted by user, stopping LandmarkDetector"
    #     self.landmark_service.unsubscribe("LandmarkDetector")
    #     #stop
    #     sys.exit(0)

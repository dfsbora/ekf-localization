
import qi
import time
import sys
import argparse
import logging



class servico(object):
    def __init__(self, session):
        self.mem_service = session.service("ALMemory")
        self.lmark_service = session.service("ALLandMarkDetection")        

    def run(self):
        try:
            while True:
                val = self.mem_service.getData("LandmarkDetected", 0)
                print("val ", val)
                time.sleep(0.5)
                print("\n")
        except KeyboardInterrupt:
            print "Interrupted by user, stopping LandmarkDetector"
            self.landmark_service.unsubscribe("LandmarkDetector")
            #stop
            sys.exit(0)       


def main(session): 
    objeto = servico(session)  
    objeto.run()




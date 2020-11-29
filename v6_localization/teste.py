#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Demonstrates how to use the ALLandMarkDetection module."""

import qi
import time
import sys
import argparse
import logging





class LandmarkDetector(object):
    """
    We first instantiate a proxy to the ALLandMarkDetection module
    Note that this module should be loaded on the robot's naoqi.
    The module output its results in ALMemory in a variable
    called "LandmarkDetected".
    We then read this ALMemory value and check whether we get
    interesting things.
    """

    def __init__(self, session):
        """
        Initialisation of qi framework and event detection.
        """
        # Get the service ALMemory.
        self.mem_service = session.service("ALMemory")
        # Connect the event callback.
        self.mem_subscriber = self.mem_service.subscriber("LandmarkDetected")
        self.mem_subscriber.signal.connect(self.on_landmark_detected)
        # Get the services ALTextToSpeech and ALLandMarkDetection.
        self.tts = session.service("ALTextToSpeech")
        self.lmark_service = session.service("ALLandMarkDetection")
        self.lmark_service.subscribe("LandmarkDetector", 500, 0.0 )
        self.got_landmark = False

    def on_landmark_detected(self, value):
        """
        Callback for event LandmarkDetected.
        """
        if value == []:  # empty value when the landmark disappears
            self.got_landmark = False
        elif not self.got_landmark:  # only speak the first time a landmark appears
            self.got_landmark = True
            print "I saw a landmark! "
            self.tts.say("I saw a landmark! ")
            # First Field = TimeStamp.
            timeStamp = value[0]
            print "TimeStamp is: " + str(timeStamp)

            # Second Field = array of mark_Info's.
            markInfoArray = value[1]
            for markInfo in markInfoArray:

                # First Field = Shape info.
                markShapeInfo = markInfo[0]

                # Second Field = Extra info (ie, mark ID).
                markExtraInfo = markInfo[1]
                print "mark  ID: %d" % (markExtraInfo[0])
                print "  alpha %.3f - beta %.3f" % (markShapeInfo[1], markShapeInfo[2])
                print "  width %.3f - height %.3f" % (markShapeInfo[3], markShapeInfo[4])

    def run(self):
        """
        Loop on, wait for events until manual interruption.
        """
        print "Starting LandmarkDetector"
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print "Interrupted by user, stopping LandmarkDetector"
            self.landmark_service.unsubscribe("LandmarkDetector")
            #stop
            sys.exit(0)



#if __name__ == "__main__":
def main():


    session = qi.Session()

    try:
        session.connect("tcp://nao.local:9559") 
        logging.debug("Connected")

    except RuntimeError:
        logging.exception("Can't connect to Naoqi")
        sys.exit(1)

    

    landmark_detector = LandmarkDetector(session)
    landmark_detector.run()
#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Demonstrates a way to localize the robot with ALLandMarkDetection"""

import qi
import time
import sys
import argparse
import math
import almath
import logging

import unboard


class LandmarkDetector(object):
    """
    We first instantiate a proxy to the ALLandMarkDetection module
    Note that this module should be loaded on the robot's naoqi.
    The module output its results in ALMemory in a variable
    called "LandmarkDetected".
    We then read this ALMemory value and check whether we get
    interesting things.
    After that we get the related position of the landmark compared to robot.
    """

    def __init__(self, session):
        """
        Initialisation of qi framework and event detection.
        """
        super(LandmarkDetector, self).__init__()

        # Get the service ALMemory.
        self.mem_service = session.service("ALMemory")
        # Connect the event callback.
        self.mem_subscriber = self.mem_service.subscriber("LandmarkDetected")
        self.mem_subscriber.signal.connect(self.on_landmark_detected)
        # Get the services ALTextToSpeech, ALLandMarkDetection and ALMotion.
        #self.tts = session.service("ALTextToSpeech")
        self.landmark_detection = session.service("ALLandMarkDetection")
        self.motion_service = session.service("ALMotion")
        self.landmark_detection.subscribe("LandmarkDetector", 500, 0.0 )
        #self.got_landmark = False
        # Set here the size of the landmark in meters.
        self.landmarkTheoreticalSize = 0.145 #in meters
        # Set here the current camera ("CameraTop" or "CameraBottom").
        self.currentCamera = "CameraTop"


    def read():
        val = self.mem_service.getData("LandmarkDetected", 0)
        print(val)

    def on_landmark_detected(self, markData):
        """
        Callback for event LandmarkDetected.
        """

        if markData == []:  # empty value when the landmark disappears
            unboard.got_landmark = False
            unboard.landmarks = None
        else: 
            unboard.got_landmark = True

            mark_info_array = markData[1]
            mark_pos_array = []


            for mark_info in mark_info_array:

                # Retrieve landmark center position in radians.
                wzCamera = mark_info[0][1]
                wyCamera = mark_info[0][2]

                # Retrieve landmark angular size in radians.
                angularSize = mark_info[0][3]

                # Compute distance to landmark.
                distanceFromCameraToLandmark = self.landmarkTheoreticalSize / ( 2 * math.tan( angularSize / 2))

                # Get current camera position in NAO space.
                transform = self.motion_service.getTransform(self.currentCamera, 2, True)
                transformList = almath.vectorFloat(transform)
                robotToCamera = almath.Transform(transformList)

                # Compute the rotation to point towards the landmark.
                cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)

                # Compute the translation to reach the landmark.
                cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

                # Combine all transformations to get the landmark position in NAO space.
                robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform *cameraToLandmarkTranslationTransform

                landmark_x = robotToLandmark.r1_c4
                landmark_y = robotToLandmark.r2_c4
                
                mark_pos = [landmark_x, landmark_y]
                mark_pos_array.append(mark_pos)


            logging.debug("landmarks")
            logging.debug(mark_pos_array)
            unboard.landmarks = mark_pos_array




    def run(self):
        """
        Loop on, wait for events until manual interruption.
        """
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print "Interrupted by user, stopping LandmarkDetector"
            self.landmark_detection.unsubscribe("LandmarkDetector")
            #stop
            sys.exit(0)


session = qi.Session()

def main():

    landmark_detector = LandmarkDetector(session)
    landmark_detector.run()

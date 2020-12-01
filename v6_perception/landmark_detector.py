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
        self.landmark_x = None
        self.landmark_y = None
        self.landmark_z = None


    def read():
        val = self.mem_service.getData("LandmarkDetected", 0)
        print(val)

    def on_landmark_detected(self, markData):
        """
        Callback for event LandmarkDetected.
        """

        if markData == []:  # empty value when the landmark disappears
            # self.got_landmark = False
            # self.landmark_x = None
            # self.landmark_y = None
            # self.landmark_z = None

            unboard.got_landmark = False
            unboard.landmark_x = None
            unboard.landmark_y = None
            unboard.landmark_z = None
        else: 
            unboard.got_landmark = True
            #print "I saw a landmark! "
            #self.tts.say("I saw a landmark! ")

            # Retrieve landmark center position in radians.
            wzCamera = markData[1][0][0][1]
            wyCamera = markData[1][0][0][2]

            # Retrieve landmark angular size in radians.
            angularSize = markData[1][0][0][3]

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

            unboard.landmark_x = robotToLandmark.r1_c4
            unboard.landmark_y = robotToLandmark.r2_c4
            unboard.landmark_z = robotToLandmark.r3_c4
            print("loop done")




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
            self.landmark_detection.unsubscribe("LandmarkDetector")
            #stop
            sys.exit(0)


session = qi.Session()

def main():

    landmark_detector = LandmarkDetector(session)
    landmark_detector.run()

#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import qi
import time
import sys
import argparse
import math
import almath
import logging

import unboard


class LandmarkDetector(object):
    """ Implements a NAOmark detector

    Intantiate a proxy to the ALLandMarkDetection module
    Read ALMemory "LandmarkDetected" value and check whether a landmark was detected
    After that write on unboard the position of landmark in relation to robot

    Parameters
    ----------  

    session : 
        session

    """

    def __init__(self, session):
        """
        Initialisation of qi framework, services and event detection.
        """
        super(LandmarkDetector, self).__init__()

        # Set here the size of the landmark in meters.
        self.landmarkTheoreticalSize = 0.145 #printed naomarks size
        # Set here the current camera ("CameraTop" or "CameraBottom").
        self.currentCamera = "CameraTop"

        # Get the service ALMemory.
        self.mem_service = session.service("ALMemory")

        # Get the services ALLandMarkDetection and ALMotion.
        #self.tts = session.service("ALTextToSpeech")
        self.landmark_detection = session.service("ALLandMarkDetection")
        self.motion_service = session.service("ALMotion")
        self.landmark_detection.subscribe("LandmarkDetector", 500, 0.0 )

                # Connect the event callback.
        self.mem_subscriber = self.mem_service.subscriber("LandmarkDetected")
        self.mem_subscriber.signal.connect(self.on_landmark_detected)



    def on_landmark_detected(self, markData):
        """
        Callback for event LandmarkDetected.
        """

        # Check if landmark was detected
        if markData == []:  
            unboard.got_landmark = False
            unboard.landmarks = None

        else: 
            unboard.got_landmark = True

            mark_info_array = markData[1]
            mark_pos_array = []

            # Loop over each detected feature
            for mark_info in mark_info_array:

                # Retrieve landmark id (number)
                landmark_id = mark_info[1]

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
                
                mark_pos = [landmark_id, landmark_x, landmark_y]
                mark_pos_array.append(mark_pos)
                #logging.debug("x: %s", landmark_x)
                #logging.debug("y: %s", landmark_y)

            #Write on unboard
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


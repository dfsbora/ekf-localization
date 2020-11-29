#!/usr/bin/env python

import vision_definitions
import numpy as np
import cv2
import time
import random
import os
import logging
import qi
import argparse

def alImage2npImage(image):

            imgWidth = image[0]
            imgHeight = image[1]
            imgChannels = image[2]
            npImg = np.reshape(image[6], (imgHeight, imgWidth, imgChannels))
            return npImg

def main():

    # Get the service ALVideoDevice.
    video_service = session.service("ALVideoDevice")

    # Register a Generic Video Module
    resolution = vision_definitions.kQVGA
    colorSpace = vision_definitions.kBGRColorSpace
    fps = 5

    topCamera = video_service.subscribeCamera(
        "python_top" + str(random.randint(1, 10)), 0, resolution, colorSpace,
        fps)
    botCamera = video_service.subscribeCamera(
        "python_bot" + str(random.randint(1, 10)), 1, resolution, colorSpace,
        fps)

    if not os.path.exists("images"):
        os.makedirs("images")

    i=0
    print("Pressione 'q' para sair e outra tecla pra capturar imagem")
    # Get ALImages of top camera and bottom camera
    while (True):

        s=raw_input()

        if s == "q":
            break

        topNAOImage = video_service.getImageRemote(topCamera)
        if (topNAOImage == None):
            logging.error("No topImage")

        botNAOImage = video_service.getImageRemote(botCamera)
        if (botNAOImage == None):
            logging.error("No botImage")


        topImage = alImage2npImage(topNAOImage)
        botImage = alImage2npImage(botNAOImage)

        filename = "images/"+str(i)+".jpg"
        cv2.imwrite(filename, topImage)
        print("Salvou imagem {}".format(i))
        i+=1


    video_service.unsubscribe(topCamera)
    video_service.unsubscribe(botCamera)



# Establishing connection with the robot
parser = argparse.ArgumentParser()
parser.add_argument(
    "--ip",
    type=str,
    #default="127.0.0.1",
    default="192.168.25.9",
    help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")
parser.add_argument("--simulation", type=bool, default=False, help="Whether code is running is simulation or a real robot")

args = parser.parse_args()
session = qi.Session()

#parse naoqi stuff
try:
    session.connect("tcp://" + args.ip + ":" + str(args.port))
except RuntimeError:
    logging.exception(
        "Can't connect to Naoqi at ip \"" + args.ip + "\" on port " +
        str(args.port) + ".\n"
        "Please check your script arguments. Run with -h option for help.")
    sys.exit(1)

main()
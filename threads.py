#!/usr/bin/env python

import qi
import argparse
import sys
import os
import threading

#logging information
#https://realpython.com/python-logging/
import logging
#change logging level accordingly
logging.basicConfig(level=logging.INFO)

# Adding localization code modules to library path
sys.path.append(os.path.join(sys.path[0], 'v6_action'))
sys.path.append(os.path.join(sys.path[0], 'v6_localization'))


# Importing localization code modules
import action
import localization


session = qi.Session()

try:
    session.connect("tcp://nao.local:9559") 
    logging.debug("Connected")

except RuntimeError:
    logging.exception("Can't connect to Naoqi")
    sys.exit(1)

action.session = session
localization.session = session


try:
    logging.info("Starting action thread ...")
    action_thread = threading.Thread(target=action.main)
    action_thread.start()
    logging.debug("Action thread started!")

    
    logging.info("Starting localization thread ...")
    localization_thread = threading.Thread(target=localization.main)
    localization_thread.start()
    logging.debug("Localization thread started!")


    action_thread.join()
    localization_thread.join()

except:
    sys.exit(1)
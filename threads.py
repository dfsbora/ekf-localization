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
import teste


session = qi.Session()

try:
    session.connect("tcp://nao.local:9559") 
    logging.debug("Connected")

except RuntimeError:
    logging.exception("Can't connect to Naoqi")
    sys.exit(1)

action.session = session
teste.session = session


try:
    logging.info("Starting action thread ...")
    action_thread = threading.Thread(target=action.main)
    action_thread.start()
    logging.debug("Action thread started!")

    
    logging.info("Starting localization thread ...")
    teste_thread = threading.Thread(target=teste.main)
    teste_thread.start()
    logging.debug("Localization thread started!")


    action_thread.join()
    teste_thread.join()

except:
    sys.exit(1)

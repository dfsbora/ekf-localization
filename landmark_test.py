import time
from naoqi import ALProxy
import numpy as np

# Replace this with your robot's IP address
robotIP = "nao.local"
PORT = 9559
    
# Create a proxy to ALLandMarkDetection
try:
    landmark_proxy = ALProxy("ALLandMarkDetection", robotIP, PORT)
except Exception, e:
    print "Error when creating landmark detection proxy:"
    print str(e)
    exit(1)

# Subscribe to the ALLandMarkDetection proxy
# Writing in ALMemory with period
period = 500
landmark_proxy.subscribe("Landmark", period, 0.0 )


# ALMemory variable
mem_value = "LandmarkDetected"

# Create a proxy to ALMemory
try:
    mem_proxy = ALProxy("ALMemory","localhost",9559)
except Exception, e:
    print "Error when creating memory proxy:"
    print str(e)
    exit(1)

print "Creating landmark detection proxy"


positions = []
infos = []
camera = []

# Read the memValue and check detected landmarks
while True:

    s=raw_input()

    if s == "q":
        break

    positions.append(s)

    #time.sleep(0.5)
    val = mem_proxy.getData(mem_value, 0)

    print ""
    print "\*****"
    print ""



    # Check whether we got a valid output: a list with two fields.
    if(val and isinstance(val, list) and len(val) >= 2):
        time_stamp = val[0]
        mark_info_array = val[1]
        camera_pose = val[2]

        infos.append(mark_info_array)
        camera.append(camera_pose)

        try:
            # Get info on each detected mark.
            for mark_info in mark_info_array:
                # First Field = Shape info.
                mark_shape_info = mark_info[0]
                # Second Field = Extra info (i.e., mark ID).
                mark_id = mark_info[1]
                # Print Mark information.
                # print "mark    ID: %d" % (mark_id[0])
                # print "    alpha %.3f - beta %.3f" % (mark_shape_info[1], mark_shape_info[2])
                # print "    width %.3f - height %.3f" % (mark_shape_info[3], mark_shape_info[4])
                # print "    heading %.3f" % (mark_shape_info[5])

                distance = 0.134*mark_shape_info[3]**(-1.04)
                angle = np.degrees(mark_shape_info[1])
                print "distance %.2f" % (distance)
                print "angle %.1f" % (angle)

        except Exception, e:
            print "Landmarks detected, but it seems getData is invalid. ALValue ="
            print val
            print "Error msg %s" % (str(e))

        # try:

        #     print "Positionx, y, z: %.2f , %.2f ,%.2f " % (camera_pose[0], camera_pose[1], camera_pose[2])
        #     print ": %.2f , %.2f ,%.2f " % (camera_pose[3], camera_pose[4], camera_pose[5])
 
        # except Exception, e:
        #     print "Landmarks detected, but it seems getData is invalid. ALValue ="
        #     print val
        #     print "Error msg %s" % (str(e))

    else:
        print "Error with getData. ALValue = %s" % (str(val))

print "Writing file"

for p,i,c in zip(positions, infos, camera):
    print ""
    print "\*****"
    print ""
    print p
    print '\n'
    print i
    print '\n'
    print c



# Unsubscribe from the module.
landmark_proxy.unsubscribe("Landmark")
print "Test terminated successfully."

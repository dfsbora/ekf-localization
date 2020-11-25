import time
from naoqi import ALProxy

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
# This means that the module will write in ALMemory with
# the given period below
period = 500
landmark_proxy.subscribe("Landmark", period, 0.0 )





# ALMemory variable where the ALLandMarkdetection module
# outputs its results
mem_value = "LandmarkDetected"

# Create a proxy to ALMemory
try:
    mem_proxy = ALProxy("ALMemory","localhost",9559)
except Exception, e:
    print "Error when creating memory proxy:"
    print str(e)
    exit(1)

print "Creating landmark detection proxy"

# A simple loop that reads the memValue and checks
# whether landmarks are detected.
for i in range(0, 20):
    time.sleep(0.5)
    val = mem_proxy.getData(mem_value, 0)
    print ""
    print "\*****"
    print ""

    # Check whether we got a valid output: a list with two fields.
    if(val and isinstance(val, list) and len(val) >= 2):
        # We detected landmarks !
        # For each mark, we can read its shape info and ID.
        # First Field = TimeStamp.
        timeStamp = val[0]
        # Second Field = array of Mark_Info's.
        markInfoArray = val[1]

        try:
            # Browse the markInfoArray to get info on each detected mark.
            for markInfo in markInfoArray:
                # First Field = Shape info.
                markShapeInfo = markInfo[0]
                # Second Field = Extra info (i.e., mark ID).
                markExtraInfo = markInfo[1]
                # Print Mark information.
                print "mark    ID: %d" % (markExtraInfo[0])
                print "    alpha %.3f - beta %.3f" % (markShapeInfo[1], markShapeInfo[2])
                print "    width %.3f - height %.3f" % (markShapeInfo[3], markShapeInfo[4])
        except Exception, e:
            print "Landmarks detected, but it seems getData is invalid. ALValue ="
            print val
            print "Error msg %s" % (str(e))
    else:
        print "Error with getData. ALValue = %s" % (str(val))

# Unsubscribe from the module.
landmark_proxy.unsubscribe("Landmark")
print "Test terminated successfully."

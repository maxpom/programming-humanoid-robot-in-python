# -*- encoding: UTF-8 -*- 
# This test demonstrates how to use the ALFaceDetection module.
# Note that you might not have this module depending on your distribution
#
# - We first instantiate a proxy to the ALFaceDetection module
#     Note that this module should be loaded on the robot's naoqi.
#     The module output its results in ALMemory in a variable
#     called "FaceDetected"

# - We then read this ALMemory value and check whether we get
#   interesting things.

import time

from naoqi import ALProxy

IP = "nao2.local"  # Replace here with your NaoQi's IP address.
PORT = 9559


"""
# Create a proxy to ALFaceDetection
try:
  blobProxy = ALProxy("ALColorBlobDetection", IP, PORT)
except Exception, e:
  print "Error when creating blob detection proxy:"
  print str(e)
  exit(1)

try:
  managerProxy = ALProxy("ALBehaviorManager", IP, PORT)
except Exception, e:
  print "Error when creating manager proxy:"
  print str(e)
  exit(1)

try:
  videoProxy = ALProxy("ALVideoDevice", IP, PORT)
except Exception, e:
  print "Error when creating video proxy:"
  print str(e)
  exit(1)





#videoProxy.setActiveCamera(0)
#subscribeId = videoProxy.subscribe("test")


try:
  redBallProxy = ALProxy("ALRedBallDetection", IP, PORT)
except Exception, e:
  print "Error when creating video proxy:"
  print str(e)
  exit(1)
"""

try:
  tracker = ALProxy("ALTracker", ip, port)
except Exception, e:
  print "Error when creating video proxy:"
  print str(e)
  exit(1)


def track_ball(self, target_name, diameter):
        self.tracker.registerTarget(target_name, diameter)
        time.sleep(3.0)
        temp = self.tracker.getRegisteredTargets()
        time.sleep(1.0)
        print temp
        temp2 = self.tracker.getMode()
        print "in mode: " + temp2
        self.motion.setStiffnesses("Head", 1.0)
        self.tracker.track(target_name)
        time.sleep(10.0)
        self.tracker.stopTracker()
        self.motion.setStiffnesses("Head", 0)

track_ball()


print "test"

"""
# Subscribe to the ALFaceDetection proxy
# This means that the module will write in ALMemory with
# the given period below
period = 500
blobProxy.subscribe("Test_blob", period, 0.0 )

# ALMemory variable where the ALFacedetection modules
# outputs its results
memValue = "ALTracker/ColorBlobDetected"

# Create a proxy to ALMemory
try:
  memoryProxy = ALProxy("ALMemory", IP, PORT)
except Exception, e:
  print "Error when creating memory proxy:"
  print str(e)
  exit(1)


r,g,b=255,127,36
colortreshhold=255 #keine Ahnung ob hoch gut ist
blobProxy.setColor(r,g,b,colortreshhold)

minSize=2000 #in Pixel
span=0.05 #in Meters
shape="Circle"
blobProxy.setObjectProperties(minSize,span,shape)

if( blobProxy.getAutoExposure()==False):
    blobProxy.setAutoExposure=True

# A simple loop that reads the memValue and checks whether faces are detected.
for i in range(0, 1):
  time.sleep(0.5)
  val = memoryProxy.getData(memValue)

  print ""
  print "*****"
  print ""

  # Check whether we got a valid output.
  if(val and isinstance(val, list) and len(val) >= 1):

    # We detected faces !
    # For each face, we can read its shape info and ID.

    # First Field = TimeStamp.
    timeStamp = val[0]

    # Second Field = array of face_Info's.
    faceInfoArray = val[1]

    try:
      # Browse the faceInfoArray to get info on each detected face.
      for j in range( len(faceInfoArray)-1 ):
        faceInfo = faceInfoArray[j]

        # First Field = Shape info.
        faceShapeInfo = faceInfo[0]

        # Second Field = Extra info (empty for now).
        faceExtraInfo = faceInfo[1]

        print "  alpha %.3f - beta %.3f" % (faceShapeInfo[1], faceShapeInfo[2])
        print "  width %.3f - height %.3f" % (faceShapeInfo[3], faceShapeInfo[4])

    except Exception, e:
      print "faces detected, but it seems getData is invalid. ALValue ="
      print val
      print "Error msg %s" % (str(e))
  else:
    print "No face detected"
"""

# Unsubscribe the module.
tracker.unsubscribe("Test_blob")
#managerProxy.stopAllBehaviors()
print "Test terminated successfully."
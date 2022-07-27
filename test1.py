
# -*- encoding: UTF-8 -*-

import time
from naoqi import ALProxy

def main():
    IP = "nao2.local"
    PORT = 9559
    diameterOfBall = 0.1
    effector = "RArm"

    print "Connecting to", IP, "with port", PORT

    motion = ALProxy("ALMotion", IP, PORT)
    posture = ALProxy("ALRobotPosture", IP, PORT)
    tracker = ALProxy("ALTracker", IP, PORT)

    # First, wake up.
    motion.wakeUp()

    fractionMaxSpeed = 0.8
    # Go to posture stand
    posture.goToPosture("StandInit", fractionMaxSpeed)

    # Add target to track.
    targetName = "RedBall"
    diameterOfBall = ballSize
    tracker.registerTarget(targetName, diameterOfBall)

    # set mode
    mode = "Head"
    tracker.setMode(mode)

    # set effector
    tracker.setEffector(effector)

    # Then, start tracker.
    tracker.track(targetName)

    print "ALTracker successfully started, now show a red ball to robot!"
    print "Use Ctrl+c to stop this script."

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print
        print "Interrupted by user"
        print "Stopping..."

    # Stop tracker, go to posture Sit.
    tracker.stopTracker()
    tracker.unregisterAllTargets()
    tracker.setEffector("None")
    posture.goToPosture("Sit", fractionMaxSpeed)
    motion.rest()

    print "ALTracker stopped."


if __name__ == "__main__" :
    
    main()
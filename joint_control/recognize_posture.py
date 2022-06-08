'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from os import listdir
from turtle import pos
from angle_interpolation import AngleInterpolationAgent
from keyframes.rightBackToStand import rightBackToStand
from keyframes.rightBellyToStand import rightBellyToStand
from keyframes.leftBackToStand import leftBackToStand
from keyframes.leftBellyToStand import leftBellyToStand
import pickle
import numpy as np

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open('../joint_control/robot_pose.pkl', 'rb'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE

        classes = listdir('../joint_control/robot_pose_data')
        joints = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        
        
        data = []
        imu = perception.imu

        for joint in joints:
            data.append(perception.joint[joint])
    
        data.append(imu[0])
        data.append(imu[1])
        data = np.array([data])
        
        predicted = self.posture_classifier.predict(data)
        
        posture = classes[int(predicted)]
        
        if(posture=="HeadBack"):  #workaround
            posture= "Belly"

        elif(posture=="StandInit"):
            posture="Back"
        elif(posture=="Belly"):
            posture="Stand"
        

        #print("Result " + posture)

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = leftBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()

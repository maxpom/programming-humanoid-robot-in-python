'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes.wipe_forehead import wipe_forehead
import numpy as np
from keyframes.leftBellyToStand import leftBellyToStand
from keyframes.leftBackToStand import leftBackToStand
from keyframes.rightBackToStand import rightBackToStand
from keyframes.rightBellyToStand import rightBellyToStand


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.offset=0
        self.animationDone=False

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints['RHipYawPitch'] = self.target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)

        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
    

        jointname = keyframes[0]
        time = keyframes[1]
        keys = keyframes[2]

        
        if(self.offset==0):
            self.animationDone = True
            

        if(self.animationDone):
            self.offset = perception.time
            self.animationDone = False

        maxtime = 0

        for j in range(len(jointname)):
            xp=time[j]
            fp=[0]*len(xp)
            x= perception.time
            for i in range(len(xp)):  
                fp[i]=keys[j][i][0]  #only first element for spline interpolation
            target_joints[jointname[j]]=np.interp(x-self.offset,xp,fp)

            maxArray=np.amax(xp)
            if(maxArray>maxtime):   # Compute Longest joint time
                maxtime=maxArray
            
        if(perception.time-self.offset>maxtime): # Animation for Keyframe done
            self.animationDone=True 
            self.keyframes= ([],[],[])
    
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = wipe_forehead()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()

'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from math import atan2
import random
from time import time
from tkinter.font import names
from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from numpy.linalg import pinv

class InverseKinematicsAgent(ForwardKinematicsAgent):
    
    def from_trans(self,m):
        x = m[3, 0]
        y = m[3, 1]
        z = m[3, 2]
        
        theta = 0
        if m[0, 0] == 1: # rotation around x-axis
            theta = np.arctan2(m[2, 1], m[1, 1])
        elif m[1, 1] == 1: # rotation around y-axis
            theta = np.arctan2(m[0, 2], m[0, 0])
        elif m[2, 2] == 1: # rotation around z-axis
            theta = np.arctan2([0, 1], m[0, 0])
            
        return x, y, z, theta

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE

        #from inverse kinematics jacobian jupyter
        lambda_ = 1
        max_step=0.1
        
        joint_names=self.chains[effector_name]
        """
        joint_angle= []
        for joint in joint_names:
                joint_angle.append(self.perception.joint[joint])
        joints=dict(zip(joint_names,joint_angle)) #bring joints in form for forward
        print(effector_name)
        print(self.perception.joint)
        """
        theta = np.random.random(len(joint_names)-1) * 1e-5
        
        for i in range (1000):
            
            
            self.forward_kinematics(self.perception.joint)
            Ts=list()
            for joint in joint_names:
                Ts.append(self.transforms[joint])
            
          
            theta_e = 0
            if Ts[-1][0,0] == 1: # rotation around x-axis
                theta_e = np.arctan2(Ts[-1][2,1], Ts[-1][1,1])
            elif Ts[-1][1,1] == 1: # rotation around y-axis
                theta_e = np.arctan2(Ts[-1][0,2], Ts[-1][0,0])
            elif Ts[-1][2,2] == 1: # rotation around z-axis
                theta_e = np.arctan2(Ts[-1][0,1], Ts[-1][0,0])

            Te= np.matrix( [(Ts[-1][3,0]), (Ts[-1][3,1]) , (Ts[-1][3,2]) , theta_e]).T
            
            theta = 0
            if transform[0,0] == 1: # rotation around x-axis
                theta = np.arctan2(transform[2,1], transform[1,1])
            elif transform[1,1] == 1: # rotation around y-axis
                theta = np.arctan2(transform[0,2], transform[0,0])
            elif transform[2,2] == 1: # rotation around z-axis
                theta = np.arctan2(transform[0,1], transform[0,0])

            target=np.matrix([(transform[3,0]),(transform[3,1]),(transform[3,2]),theta])

            e=target - Te
            e[e>max_step] = max_step
            e[e<-max_step] = -max_step
            T= np.matrix([self.from_trans(i)for i in Ts[:]]).T
            J= Te-T
            dT= Te-T
            J[0, :] = -dT[1, :] # x
            J[1, :] = dT[0, :] # y
            J[-1, :] = 1  # angular
            d_theta = lambda_ * pinv(J) * e

            theta += np.asarray(d_theta.T)[0]

            for j in range(len(joint_names)):
        
                joint_angles.append(theta[j])

            if  np.linalg.norm(d_theta) < 1e-4:
                break


        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joints= self.inverse_kinematics(effector_name,transform)

        names=self.chains[effector_name]

        times=[[0., 3.]] * len(names)

        keys=list()
        i=0
        for name in names:
            keys.append([[self.perception.joint[name], [], [joints[i], []]]])
            i+=1

        self.keyframes = ([names], [times], [keys])  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()

'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import threading
import numpy as np
from telnetlib import SE
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

#from inverse_kinematics import InverseKinematicsAgent
from forward_kinematics import ForwardKinematicsAgent  #swapped to Forward Kinematic Agent because Inverse still dont works
from xmlrpc.server import SimpleXMLRPCServer

class ServerAgent(ForwardKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    

    def __init__(self):
        super(ServerAgent,self).__init__()
        self.server = SimpleXMLRPCServer(("127.0.0.1",8080),allow_none="True")

        
        self.server.register_function(self.get_angle)
        self.server.register_function(self.set_angle)
        self.server.register_function(self.get_posture)
        self.server.register_function(self.execute_keyframes)
        self.server.register_function(self.get_transform)
        self.server.register_function(self.set_transform)
        
        #self.server.register_instance(ServerAgent)
        
        
        s=self.server.serve_forever                       #threading for blocking and non blocking
        self.thread= threading.Thread(target=s)
        self.thread.daemon=True                         #Thread kills itself
        self.thread.start()
        print("starting Server...")

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        print("get Angle")
        return self.perception.joint[joint_name]
        # YOUR CODE HERE
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name]=angle
        print("set Angle")

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        print("get Posture")
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.start_time = -1
        self.keyframes = keyframes
        while self.start_time != -1:
            pass
        print("execute Keyframe")
        return True
        

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        print("get Transform")
        return self.transform[name]

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transform(effector_name, np.matrix(transform))
        print ("set Transform")

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()


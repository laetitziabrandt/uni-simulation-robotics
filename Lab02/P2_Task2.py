# This is a sample Python script for exercise 2 and 3.
import numpy as np
import math
import time
import transformations as t
from random import  uniform
from pynput import keyboard
from coppeliasim_zmqremoteapi_client import *
from P2_Task1 import *


keypress=''  # global variable


############################################################################
def on_press(key):
    global keypress
    try:
        keypress = key.char
    except AttributeError:
        keypress = key
############################################################################
def on_release(key):
    global keypress
    keypress = ''

################################################################################
def forward_kinematics(rob, joints):
    # joints is an array that contains each joints' position

    TCP = t.identity_matrix()

    number_joints = len(joints)
    for i in range(1, number_joints+1):
        # Rotation(joint) * Translation * Rotation
        Rq = t.rotation_matrix(joints[i-1], [0, 0, 1])
        T = t.translation_matrix(rob[i][:3])
        R = t.euler_matrix(rob[i][3], rob[i][4], rob[i][5], 'rxyz')
        TCP = t.concatenate_matrices(TCP, Rq, T, R)

    return TCP
################################################################################
if __name__ == '__main__':

    print('Program started')

    # degrees of freedom of snake robot
    dof = 6

    # kinematic structure of snake robot  - to be filled out
    snake_robot = np.array([
        [0, 0, 0, 0, 0, 0],             # WCS to 1. joint
        [0.25, 0, 0, 0, 0, -1.5708],    # 1. to 2.
        [0.25, 0, 0, 0, 0, 1.5708],     # 2. to 3.
        [0.25, 0, 0, 0, 0, -1.5708],    # 3. to 4.
        [0.25, 0, 0, 0, 0, 1.5708],     # 4. to 5.
        [0.25, 0, 0, 0, 0, -1.5708],    # 5. to 6.
        [0.25, 0, 0, 0, 0, 0]])         # 6. to TCP

    # connect to Remote API
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    # Getting Handles of the Tool Centre Point (TCP)
    handle_tcp = sim.getObjectHandle('TCP')

    jh = np.empty(dof, dtype=object)  # create an empty array for joint handles
    # getting the handles for the joints
    jh[0]=sim.getObjectHandle('./Revolute_joint1')
    jh[1]=sim.getObjectHandle('./Revolute_joint2')
    jh[2]=sim.getObjectHandle('./Revolute_joint3')
    jh[3]=sim.getObjectHandle('./Revolute_joint4')
    jh[4]=sim.getObjectHandle('./Revolute_joint5')
    jh[5]=sim.getObjectHandle('./Revolute_joint6')
    time.sleep(1)

    joints = np.empty(dof, dtype=object)  # create an empty array for the joints

    # set all joints to 0
    for i in range(dof):
        sim.setJointPosition(jh[i], 0.0)

    listener = keyboard.Listener(on_press=on_press,on_release=on_release)
    listener.start()

    while True:
        while keypress=='':  # wait for keyboard input
            pass
        if keypress == 'q':
            print("You pressed q to quit")
            break
        # set joints to random position between [0, pi/4]
        for i in range(dof):
            sim.setJointPosition(jh[i], uniform(0.0, math.pi/4))
            # sim.setJointPosition(jh[i], 0) # joints static

        angles = sim.getObjectOrientation(handle_tcp, -1)  # Try to retrieve orientation
        pos    = sim.getObjectPosition(handle_tcp, -1)  # Try to retrieve position

        # retrieve the joint positions
        for i in range(dof):
            joints[i] = sim.getJointPosition(jh[i])

        print('angles: ', angles)
        R1 = matrix_from_euler(angles)
        print('TCP Rotation: ')
        print(R1)
        print('TCP Position: ', pos)
        TCP_frame = forward_kinematics(snake_robot, joints)
        print('TCP frame: ')
        print(TCP_frame)
        rot = TCP_frame[:3, :3]
        print('Euler angles from rot matrix: ', euler_from_matrix(rot))
        print("----------------------------------------------------   ")
        # time.sleep(0.005)

        keypress = ''

    listener.stop()
    print('Program ended')

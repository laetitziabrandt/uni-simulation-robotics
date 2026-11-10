# This is a sample Python script for the exercise 2 and 3.

import numpy as np
import math
import transformations as t
import time
from random import  uniform
from pynput import keyboard
from coppeliasim_zmqremoteapi_client import *

# global variables
keypress=''
l1 = 0.25
l2 = 0.25
l3 = 0.25

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

###########################################################################
# this needs to be implemented
def inverse_jacobian_3dof(joint):
    # returning the inverse of the jacobian matrix
    q1 = joint[0]
    q2 = joint[1]
    q3 = joint[2]
    
    # First row of the matrix
    r00 = - l1*math.sin(q1) - l2*math.sin(q1 + q2) - l3*math.sin(q1 + q2 + q3)
    r01 = - l2*math.sin(q1 + q2) - l3*math.sin(q1 + q2 + q3)
    r02 = - l3*math.sin(q1 + q2 + q3)

    # Second row of the matrix
    r10 = l1*math.cos(q1) + l2*math.cos(q1 + q2) + l3*math.cos(q1 + q2 + q3)
    r11 = l2*math.cos(q1 + q2) + l3*math.cos(q1 + q2 + q3)
    r12 = l3*math.cos(q1 + q2 + q3)

    # Third row of the matrix
    r20 = 1
    r21 = 1
    r22 = 1

    jacobian = np.array([[r00, r01, r02],
                        [r10, r11, r12],
                        [r20, r21, r22]])

    inv_jacobian = np.linalg.pinv(jacobian)

    return inv_jacobian

################################################################################
if __name__ == '__main__':
    print('Program started')
    # connect to Remote API
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    # degrees of freedom of 3-DOF robot
    dof = 3
    # kinematic structure of 3-DOF robot  - to be filled out
    dx = np.array([0.0, 0.0])
    DOF3_robot = np.array([
        [0, 0, 0, 0, 0, 0],         #transformation from world coordinate system (WCS) to robot base
        [0.25, 0, 0, 0, 0, 0],      # transformation from robot base to link2
        [0.25, 0, 0, 0, 0, 0],      # transformation from link2 to link3
        [0.25, 0, 0, 0, 0, 0]])     # transformation from link3 to TCP

    defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
    sim.setInt32Param(sim.intparam_idle_fps, 0)
    client.setStepping(True)
    sim.startSimulation()

    # Getting Handles of the TCP
    handle_tcp = sim.getObjectHandle('TCP')

    jh = np.empty(dof, dtype=object)  # create an empty array for joint handles
    # getting the handles for the joints
    jh[0]=sim.getObjectHandle('./Revolute_joint1')
    jh[1]=sim.getObjectHandle('./Revolute_joint2')
    jh[2]=sim.getObjectHandle('./Revolute_joint3')

    joints = np.empty(dof, dtype=object)  # create an empty array for the joints
    # set all joints to 20 degrees
    joints[0] = 20.0 * math.pi/180.0
    joints[1] = 20.0 * math.pi/180.0
    joints[2] = 20.0 * math.pi/180.0
    for i in range(dof):
            sim.setJointPosition(jh[i], joints[i])
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while True:
        while keypress == '':  # wait for keyboard input
            client.step()     # remove coppeliasim popup
            time.sleep(0.01)  # give thread time to process input
        if keypress == 'q':
            print("You pressed q to quit")
            break
        dx = np.array([0.0, 0.0])
        delta_pos = 0.02  # in m for each simulation step
        if keypress == "u":   # move up
            dx = np.array([0.0, delta_pos])
        if keypress == "n":   # move down
            dx = np.array([0.0, -delta_pos])
        if keypress == "h":    # move left
            dx = np.array([-delta_pos, 0.0])
        if keypress == "j":    # move right
            dx = np.array([delta_pos, 0.0])
        if keypress == "s":    # stop movement
            dx = np.array([0.0, 0.0])

        inverse_jacobian = inverse_jacobian_3dof(joints)
        # x and y coords of tcp
        dx_tcp = np.array([dx[0], dx[1], 0.0])
        dq = inverse_jacobian @ dx_tcp

        # set new joint angles
        joints[0] += dq[0]
        joints[1] += dq[1]
        joints[2] += dq[2]

        # convert to degrees
        dx_tcp_deg = np.degrees(dx_tcp)

        dq_deg = np.degrees(dq)

        tcp_pos = sim.getObjectPosition(sim.getObjectHandle("./TCP"), -1)
        print("\ninitial tcp pos: ", tcp_pos[0], tcp_pos[1])

        print("jacobian:\n", inverse_jacobian)

        print("dx: ", dx_tcp_deg)
        print("dq: ", dq_deg)

        print("q1: ", math.degrees(joints[0]))
        print("q2: ", joints[1] * 180 / math.pi)
        print("q3: ", joints[2] * 180 / math.pi)

        # set all joints
        for i in range(dof):
            sim.setJointPosition(jh[i], joints[i])
        client.step()

        tcp_pos = sim.getObjectPosition(sim.getObjectHandle("./TCP"), -1)
        print("changed tcp pos: ", tcp_pos[0], tcp_pos[1])

        if sim.getSimulationState() == sim.simulation_stopped:
            break
        keypress = ''

    listener.stop()
    # Now stop the simulation:
    sim.stopSimulation()
    sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

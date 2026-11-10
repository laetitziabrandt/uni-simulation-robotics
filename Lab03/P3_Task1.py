# This is a sample Python script for the exercise 2 and 3.

import sys
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

##########################################################################
# this needs to be implemented
def inverse_kinematics_2dof(x, y, elbow_pos):
    # returning two angles, which generates the given TCP position in x and y

    alpha = math.atan2(y, x)
    beta = math.acos(( l1*l1 + x*x + y*y - l2*l2 ) / (2*l1*math.sqrt(x*x + y*y)))
    # q1 = alpha - beta

    # calculate angle phi
    numerator = ( pow(l1, 2) + pow(l2, 2 ) - ( pow(x, 2) + pow(y, 2) ) )
    denominator = (2 * l1 * l2)
    phi = math.acos(numerator/denominator)

    # calculate angle q1 and q2 based on elbow position
    q1, q2 = 0, 0
    if elbow_pos == 'down':
        q2 = math.pi - phi
        q1 = alpha - beta
    elif elbow_pos == 'up':
        q2 = -(math.pi - phi)
        q1 = alpha + beta

    return q1, q2

################################################################################
if __name__ == '__main__':
    print('Program started')
    # connect to Remote API
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    # degrees of freedom of 2-DOF robot
    dof = 2
    # kinematic structure of 2-DOF robot  - to be filled out
    dx = np.array([0.0, 0.0])
    DOF3_robot = np.array([
        [0, 0, 0, 0, 0, 0],         #transformation from world coordinate system (WCS) to robot base
        [0.25, 0, 0, 0, 0, 0],      # transformation from robot base to link2
        [0.25, 0, 0, 0, 0, 0]])     # transformation from link2 to TCP

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
    elbow_up = 0
    #time.sleep(1)
    joints = np.empty(dof, dtype=object)  # create an empty array for the joints
    # set all joints to 20 degrees
    joints[0] = 20.0 * math.pi/180.0
    joints[1] = 20.0 * math.pi/180.0
    for i in range(dof):
            sim.setJointPosition(jh[i], joints[i])
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while True:
        while keypress == '':  # wait for keyboard input
            client.step()  # remove coppeliasim popup
            time.sleep(0.01)   # give thread time to process input
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
        if keypress == "e":    #
            elbow_up = elbow_up ^ 1
            print("elbow up: ", elbow_up)

        print('') # newline

        tcp_pos = sim.getObjectPosition(sim.getObjectHandle("./TCP"), -1)
        print("initial tcp pos: ", tcp_pos[0], tcp_pos[1])

        try:
            tcp_pos_x = tcp_pos[0]+dx[0]
            tcp_pos_y = tcp_pos[1]+dx[1]
            r = math.hypot(tcp_pos_x, tcp_pos_y)
            beta = math.acos((l1 * l1 + tcp_pos_x * tcp_pos_x + tcp_pos_y * tcp_pos_y - l2 * l2) /
                             (2 * l1 * math.sqrt(tcp_pos_x * tcp_pos_x + tcp_pos_y * tcp_pos_y)))
        except ValueError:
            sim.stopSimulation()
            sys.exit("Error! TCP outside of workspace or singularity reached.")
        else:
            if elbow_up:
                joints[0], joints[1] = inverse_kinematics_2dof(tcp_pos[0]+dx[0], tcp_pos[1]+dx[1], 'up')
            else:
                joints[0], joints[1] = inverse_kinematics_2dof(tcp_pos[0] + dx[0], tcp_pos[1] + dx[1], 'down')

        print("q1: ", joints[0] * 180 / math.pi)
        print("q2: ", joints[1] * 180 / math.pi)
        print("q2 as shown in coppeliaSim: ", (joints[0]+joints[1]) * 180 / math.pi)

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

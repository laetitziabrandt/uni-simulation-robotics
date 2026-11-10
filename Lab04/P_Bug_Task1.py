# This is a sample Python script for exercise 4, variant Potential Field.
import keyboard
import numpy as np
import math
import time
from coppeliasim_zmqremoteapi_client import *

from random import uniform

################################################################################
# this function calculates the 3D unit direction vector from p1 to p2
# it returns a boolean value, whether p1 is very close to p2
# and a direction vector in unit length
# True: the direction vector is 0
# True: the direction vector is p2 - p1 with unit length
def build_unit_direction_vector(p1, p2):
    epsilon = 1e-2
    print("p1: ", p1)
    print("p2: ", p2)

    direction = np.subtract(p2, p1)
    print("direction p1 to p2: ", direction)
    # normalize direction vector
    norm_direction = np.linalg.norm(direction)

    # if bug is close to goal
    if norm_direction < epsilon:
        return True, np.zeros_like(direction) # returns same sized array with zeros

    # if not close to goal
    unit_direction = direction / norm_direction
    print("unit direction: ", unit_direction)
    return False, unit_direction

################################################################################
# calculates all (nbr_sensors) direction_vectors of unit length starting with
# sensor pointing towards starting angle
# returns all directions vectors with unit length
def generate_sensor_directions(starting_angle, nbr_sensors):
    direction_vectors = []
    increase_by_angle = 360/nbr_sensors # here: 360°/12
    angle = starting_angle
    for _ in range(nbr_sensors):
        angle_rad = math.radians(angle)  # convert degrees to radians
        x = math.cos(angle_rad)
        y = math.sin(angle_rad)
        z = 0
        direction_vectors.append([x, y, z])
        angle += increase_by_angle
    return direction_vectors

################################################################################
if __name__ == '__main__':
    # start simulation and run this program.
    #
    # IMPORTANT: for each successful call to simxStart, there
    # should be a corresponding call to simxFinish at the end!

    try:
        client = RemoteAPIClient()
        sim = client.getObject('sim')

    except:
        print('--------------------------------------------------------------')
        print('"Something went wrong connecting to the remote API client CoppeliaSim')
        print('--------------------------------------------------------------')
        print('')
    print('Connected to remote API server')
    print('Program started')
    #sim.stopSimulation()
    defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
    sim.setInt32Param(sim.intparam_idle_fps, 0)
    stepping = True  # for debugging it is recommended to set True
    stepping = False
    client.setStepping(stepping)


    dof = 2  # two degrees of freedom

    nbr_sensors = 12  # number of proximity sensors
    # calculate the directions of the proximity sensors
    direction_vectors = generate_sensor_directions(-90.0, nbr_sensors)
    print("direction_vectors: ", direction_vectors)

    # Getting Handle of the robot
    bug = sim.getObject('./Bug')
    goal_handle = sim.getObject('./Goal')

    sh = np.empty(nbr_sensors, dtype=object)  # create an empty array for sensor handles
    # getting the handles for the sensors
    for i in range(nbr_sensors):
        sh[i]=sim.getObjectHandle('./Proximity_sensor'+str(i+1))

    # Start the Simulation
    sim.startSimulation()
    print("Simulation started")
    directions = generate_sensor_directions(-90, nbr_sensors)
    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime = time.time()
    # start configuration
    start = [0, 0, 0]
    # set Bug in start position
    sim.setObjectPosition(bug, start, -1)

    cur_pos = start
    print("cur_pos: ", cur_pos)

    # goal configuration, must be not equal to start
    goal = sim.getObjectPosition(goal_handle, -1)  # [0.8, 0.4, 0]
    direction_goal = build_unit_direction_vector(start, goal)
    while True:
        # get the unit_direction_vector
        reached, direction_goal = build_unit_direction_vector(cur_pos, goal)

        if keyboard.is_pressed("q"):
            print("You pressed q, so you quit the program")
            break
        detected_distances = []  # create empty list of [index, distance] pairs
        for i in range(nbr_sensors):
            # reading out all sensor information
            """
            res: detection state (0 or 1)
            dist: distance to the detected point
            point: array of 3 numbers indicating the relative coordinates of the detected point
            obj: handle of the object that was detected
            n: normal vector (normalized) of the detected surface. Relative to the sensor reference frame
            int res, float dist, list point, int obj, list n = sim.readProximitySensor(int sensorHandle)
            """
            res, dist, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = \
            sim.readProximitySensor(sh[i])
            if res == True: # if something is detected
                # store index of sensor and distance dist
                # minus radius of bug: 5cm, since the sensor starts from the bug center
                detected_distances.append([i, dist - 0.05])
                print("Sensor ", i, " :", dist - 0.05)

        # set Bug robot to current position cur_pos moving towards goal
        cur_pos[0] = cur_pos[0]  + direction_goal[0] * 0.005   # 5 mm per iteration
        cur_pos[1] = cur_pos[1]  + direction_goal[1] * 0.005   # 5 mm per iteration
        sim.setObjectPosition(bug, cur_pos, -1)
        print("current position: ", cur_pos)
        if stepping:
            sim.step() # execute one simulation step

    # Stop the Simulation
    sim.stopSimulation()
    sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)
    print("Simulation stopped")
    sim.setObjectPosition(bug, start, -1)
    print('Program ended')

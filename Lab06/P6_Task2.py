# This is a sample Python script for this exercise.

from coppeliasim_zmqremoteapi_client import *

#######################################################################
def visualizePath(path):
    global lineContainer
    global initPos
    sim.addDrawingObjectItem(lineContainer,None)
    if path is not None:
        pc=len(path)//3
        for i in range(len(path)//3):
            lineDat=[path[3*i+0],path[3*i+1],initPos[2]]
            sim.addDrawingObjectItem(lineContainer,lineDat)


###############################################################################################################################
if __name__ == '__main__':
    print('Program started')
    client = RemoteAPIClient()
    sim = client.require('sim')
    # see OMPL API https://manual.coppeliarobotics.com/en/simOMPL.htm#compute
    simOMPL = client.require('simOMPL')

    stepping = True
    client.setStepping(stepping)
    sim.startSimulation()
    print("Simulation started")

    global lineContainer
    global initPos
    lineContainer=sim.addDrawingObject(sim.drawing_linestrip,3,0,-1,0,[0.2,0.2,0.2])
    path = None
    robotHandle = sim.getObject('./StartConfiguration')

    # your code starts here...
    goalHandle = sim.getObject('./GoalConfiguration')
    initPos = sim.getObjectPosition(robotHandle,-1)
    goalPos = sim.getObjectPosition(goalHandle,-1)
    initRot = sim.getObjectOrientation(robotHandle,-1)
    goalRot = sim.getObjectOrientation(goalHandle,-1)
    t = simOMPL.createTask('t')

    # dubins state space -> non-holonomic w/ min turning radius constraint (suitable for car-like robots)
    ss = [simOMPL.createStateSpace('2d', simOMPL.StateSpaceType.dubins, robotHandle, [-0.5, -0.5], [0.5, 0.5], 1)]
    simOMPL.setStateValidityCheckingResolution(t, 0.002) # provides finer collision checks along path
    simOMPL.setDubinsParams(ss[0], 0.05, True) # state space ss, turning radius, backward = True (allows backward motions)

    simOMPL.setStateSpace(t,ss)
    # RRT, RRTstar, PRM, PRMstar, TRRT etc
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRT)
    simOMPL.setCollisionPairs(t,[sim.getObject('./StartConfiguration/Car'), sim.handle_all])

    startPos = [initPos[0],initPos[1],initRot[2]]
    endPos = [goalPos[0],goalPos[1],goalRot[2]]
    simOMPL.setStartState(t,startPos)
    simOMPL.setGoalState(t,endPos)
    simOMPL.setup(t)
    r,path = simOMPL.compute(t,4,-1,800)
    print(simOMPL.hasSolution(t))
    #print(path)
    #print("Path length : ", len(path)//3)

    if path != None:
        visualizePath(path)
        # Simply jump through the path points, no interpolation here:
        for i in range(len(path)//6-1):
            pos = [path[6*i+0],path[6*i+1],initPos[2]]
            #orient = [initOrient[0],initOrient[1],path[3*i+2]]
            orient = [0,0,path[6*i+2]]  # needs to be adopted
            sim.setObjectPosition(robotHandle,sim.handle_world,pos)
            sim.setObjectOrientation(robotHandle,sim.handle_world,orient)
            client.step()
    else:
        pos = sim.getObjectPosition(robotHandle,-1)
        orient = sim.getObjectOrientation(robotHandle,-1)
        for i in range(80):
            print(i)
            pos[0] = pos[0] + 0.02
            sim.setObjectPosition(robotHandle, -1, pos)
            sim.setObjectOrientation(robotHandle, sim.handle_world, orient)
            client.step()

    # draw the path?
    sim.addDrawingObjectItem(lineContainer, None)
    sim.stopSimulation()
    print("Simulation stopped")

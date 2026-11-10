import time
from pynput import keyboard

from coppeliasim_zmqremoteapi_client import *
from bezier_path import *

keypress=''
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
###############################################################################################################################
if __name__ == '__main__':
    print('Program started')
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    sim.startSimulation()
    print("Simulation started")

    nbr = 4  # number of control points
    cp = np.empty(nbr, dtype=object)  # create an empty array for control point handles
    p = np.empty(nbr, dtype=object)  # create an empty array for control points
    cuboid = sim.getObjectHandle('/Cuboid')
    pos = sim.getObjectPosition(cuboid, -1)
    z = pos[2]

    # your code from here .....

    # getting the handles for the control points
    for i in range(nbr):
        cp[i] = sim.getObjectHandle('./CP[' + str(i) + ']')

    # set points
    for i in range(nbr):
        p[i] = sim.getObjectPosition(cp[i], -1)

    # creating BezierSpline2D instance
    bezier = BezierSpline2D(p[0], p[1], p[2], p[3])

    # setting number of points in the path
    nbr_path_pts = bezier.number_pts = 20

    # calculating bezier path
    # we don't use c and s, and path simply contains x and y
    path, x, y, yaw, c, s = bezier.calc_bezier_path()

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    print("Press 's' to move the cuboid")
    print("Press 'q' to quit")

    robotTrace = False
    while True:
        while keypress == '':   # wait for keyboard input
            client.step()       # remove coppeliasim popup
            time.sleep(0.01)    # give thread time to process input
        if keypress == 'q':
            print("You pressed q to quit")
            break
        if keypress == 's':
            if robotTrace:
                time.sleep(0.1)
                scriptHandle = sim.getScript(sim.scripttype_childscript, cuboid)
                sim.callScriptFunction('redraw', scriptHandle)
            for i in range(nbr_path_pts):
                time.sleep(0.04)
                sim.setObjectPosition(cuboid, [x[i], y[i], z+0.01])
                sim.setObjectOrientation(cuboid, [0, 0, yaw[i]])
            robotTrace = True
    sim.stopSimulation()
    print("Simulation stopped")


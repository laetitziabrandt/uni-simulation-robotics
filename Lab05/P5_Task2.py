import math
from coppeliasim_zmqremoteapi_client import *
from PySide6.QtWidgets import (QLineEdit, QPushButton, QApplication,
                               QVBoxLayout, QDialog, QGridLayout)
import sys

# max steering angle
max_inner_angle = math.radians(30)
max_speed = 20
###############################################################################################################################
def turn(angle, wB, tW, lwheel, rwheel):
    # wB = wheel base
    # tW = tire width, already halved.
    # Formula : angle = atan2((wB*math.tan(angle))/(wB+-0.5*tW*math.tan(angle)))
    d = 0.5 * tW
    left_angle = math.atan2((wB * math.tan(angle)), (wB - d * math.tan(angle)))
    right_angle = math.atan2((wB * math.tan(angle)), (wB + d * math.tan(angle)))
    # print("Left Angle : ", left_angle)
    # print("Right Angle : ", right_angle)
    return left_angle, right_angle


###############################################################################################################################
class Form(QDialog):

    def __init__(self, fl_steer, fr_steer, fl_drive, fr_drive, length, diameter, parent=None):
        super(Form, self).__init__(parent)
        self.fl_steer = fl_steer
        self.fr_steer = fr_steer
        self.fl_drive = fl_drive
        self.fr_drive = fr_drive
        self.length = length
        self.diameter = diameter
        self.speed = 1  # Speed of the forward/backwards
        self.innerangle = 0.0  # Current angle of the Ackermann wheel.
        self.anglechange = 5.0  # Angle change for every left/right command.

        # Buttons
        self.buttonLeft = QPushButton('←')
        self.buttonRight = QPushButton('→')
        self.buttonFront = QPushButton('↑')
        self.buttonBack = QPushButton('↓')
        self.buttonStart = QPushButton('Start simulation')
        self.buttonEnd = QPushButton('Stop simulation')
        self.buttonStop = QPushButton('Stop')

        # Layout
        layout = QGridLayout()

        layout.addWidget(self.buttonLeft, 3, 2)
        layout.addWidget(self.buttonRight, 3, 4)
        layout.addWidget(self.buttonFront, 2, 3)
        layout.addWidget(self.buttonBack, 3, 3)
        layout.addWidget(self.buttonStart, 6, 3)
        layout.addWidget(self.buttonStop, 5, 3)
        layout.addWidget(self.buttonEnd, 7, 3)

        self.setLayout(layout)

        self.buttonLeft.clicked.connect(self.goLeft)
        self.buttonRight.clicked.connect(self.goRight)
        self.buttonFront.clicked.connect(self.goForwards)
        self.buttonBack.clicked.connect(self.goBackwards)
        self.buttonStart.clicked.connect(self.startSim)
        self.buttonStop.clicked.connect(self.stop)
        self.buttonEnd.clicked.connect(self.endSim)

    def goLeft(self):
        print("Going left")

        # calc the inner angle
        self.innerangle += math.radians(self.anglechange)
        print("Inner angle: ", self.innerangle)
        # clamp the angle
        self.innerangle = min(self.innerangle, max_inner_angle)

        # get angles for left and right wheel
        lwheel_angle = sim.getJointPosition(self.fl_steer)
        rwheel_angle = sim.getJointPosition(self.fr_steer)
        print("Left Wheel : ", lwheel_angle)
        print("Right Wheel : ", rwheel_angle)

        # final wheel angle for both wheels
        wheel_angles = turn(self.innerangle, self.length, self.diameter, lwheel_angle, rwheel_angle)

        sim.setJointTargetPosition(self.fl_steer, wheel_angles[0])
        sim.setJointTargetPosition(self.fr_steer, wheel_angles[1])

    def goRight(self):
        print("Going right")

        self.innerangle -= math.radians(self.anglechange)
        print("Inner angle: ", self.innerangle)
        self.innerangle = max(self.innerangle, -max_inner_angle)

        lwheel_angle = -(sim.getJointPosition(self.fl_steer))
        rwheel_angle = -(sim.getJointPosition(self.fr_steer))
        print("Left Wheel : ", lwheel_angle)
        print("Right Wheel : ", rwheel_angle)

        wheel_angles = turn(self.innerangle, self.length, self.diameter, lwheel_angle, rwheel_angle)
        print("Wheel angles : [Left - ", math.degrees(wheel_angles[0]), ", Right - ", math.degrees(wheel_angles[1]),
              "]")
        sim.setJointTargetPosition(self.fl_steer, wheel_angles[0])
        sim.setJointTargetPosition(self.fr_steer, wheel_angles[1])

    def goForwards(self):
        print("Going forwards")
        # increase speed by 1
        self.speed += 1
        # cap at max speed
        self.speed = min(self.speed, max_speed)
        sim.setJointTargetVelocity(self.fl_drive, self.speed)
        sim.setJointTargetVelocity(self.fr_drive, self.speed)

    def goBackwards(self):
        print("Going backwards")
        self.speed -= 1
        self.speed = max(self.speed, -max_speed)
        sim.setJointTargetVelocity(self.fl_drive, self.speed)
        sim.setJointTargetVelocity(self.fr_drive, self.speed)

    def startSim(self):
        status_code_start = sim.startSimulation()
        print("Simulation started")

    def stop(self):
        self.speed = 0
        sim.setJointTargetVelocity(self.fl_drive, 0.0)
        sim.setJointTargetVelocity(self.fr_drive, 0.0)

    def endSim(self):
        sim.setJointTargetVelocity(self.fl_drive, 0.0)
        sim.setJointTargetVelocity(self.fr_drive, 0.0)
        print("Simulation stopped")
        sim.stopSimulation()


###############################################################################################################################
if __name__ == '__main__':
    print('Program started')
    client = RemoteAPIClient()
    # Get objects
    sim = client.getObject('sim')
    # The wheels
    fl_drive = sim.getObjectHandle('./motorLeft')
    fr_drive = sim.getObjectHandle('./motorRight')
    fl_steer = sim.getObjectHandle('./steeringLeft')
    fr_steer = sim.getObjectHandle('./steeringRight')
    length = 2.5772  # From construction.
    width = 2*0.75515  # From construction

    app = QApplication(sys.argv)
    form = Form(fl_steer, fr_steer, fl_drive, fr_drive, length, width)
    form.show()
    app.exec()
    sim.stopSimulation()
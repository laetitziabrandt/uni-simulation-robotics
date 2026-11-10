#!/usr/bin/env python3
# coding: utf-8
import sys
from coppeliasim_zmqremoteapi_client import *

# this is the module for our GUI
from PySide6.QtWidgets import (QLineEdit, QPushButton, QApplication,
                               QVBoxLayout, QDialog, QGridLayout)
##############################################################################
# This is the class of our GUI. In the main we create an Instance of this class.
class Form(QDialog):

    def __init__(self, leftMotor, rightMotor, parent=None):
        super(Form, self).__init__(parent)

        # Initialize client, motors and speed
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.speed = 1.0

        # Create button objects
        self.buttonForward = QPushButton("↑")
        self.buttonBackward = QPushButton("↓")
        self.buttonLeft = QPushButton("←")
        self.buttonRight = QPushButton("→")
        self.buttonStart = QPushButton("Start")
        self.buttonStop = QPushButton("Stop")
        self.buttonSpinL = QPushButton("Spin left")
        self.buttonSpinR = QPushButton("Spin right")

        # Create the layout
        layout = QGridLayout()
        # Order the buttons in the layout first number is the row, second is the column
        layout.addWidget(self.buttonForward, 0, 1)
        layout.addWidget(self.buttonBackward, 2, 1)
        layout.addWidget(self.buttonLeft, 2, 0)
        layout.addWidget(self.buttonRight, 2, 2)
        layout.addWidget(self.buttonStart, 4, 1)
        layout.addWidget(self.buttonStop, 5, 1)
        layout.addWidget(self.buttonSpinL, 0, 0)
        layout.addWidget(self.buttonSpinR, 0, 2)

        # Set dialog layout
        self.setLayout(layout)
        # Add the methods to the buttons that should get executed when you press on them
        self.buttonStart.clicked.connect(self.startSim)
        self.buttonStop.clicked.connect(self.stopSim)
        self.buttonForward.clicked.connect(self.forward)
        self.buttonBackward.clicked.connect(self.backward)
        self.buttonLeft.clicked.connect(self.left)
        self.buttonRight.clicked.connect(self.right)
        self.buttonSpinL.clicked.connect(self.spinLeft)
        self.buttonSpinR.clicked.connect(self.spinRight)

    # This method starts the simulation
    def startSim(self):
        sim.setJointTargetVelocity(self.leftMotor, 0.0)
        sim.setJointTargetVelocity(self.rightMotor, 0.0)
        sim.startSimulation()
        print("Simulation started")

    # Stop Simulation
    def stopSim(self):
        # Stop the Simulation
        sim.setJointTargetVelocity(self.leftMotor, 0.0)
        sim.setJointTargetVelocity(self.rightMotor, 0.0)
        print("Simulation stopped")

    # Sets BubbleRob motors speed to drive forward
    def forward(self):
        sim.setJointTargetVelocity(self.leftMotor, self.speed)
        sim.setJointTargetVelocity(self.rightMotor, self.speed)
        #self.speed = 1.0

    # Sets BubbleRob motors speed to drive backward
    def backward(self):
        sim.setJointTargetVelocity(self.leftMotor, -self.speed)
        sim.setJointTargetVelocity(self.rightMotor, -self.speed)
        #self.speed = -1.0

    # Sets BubbleRob motors speed to drive left
    def left(self):
        sim.setJointTargetVelocity(self.leftMotor, 1.0*self.speed)
        sim.setJointTargetVelocity(self.rightMotor, 2.0*self.speed)
    # Sets BubbleRob motors speed to drive right
    def right(self):
        sim.setJointTargetVelocity(self.leftMotor, 2.0*self.speed)
        sim.setJointTargetVelocity(self.rightMotor, 1.0*self.speed)

    # Sets BubbleRob motors speed to spin to the left
    def spinLeft(self):
        sim.setJointTargetVelocity(self.leftMotor, -self.speed)
        sim.setJointTargetVelocity(self.rightMotor, self.speed)

    # Sets BubbleRob motors speed to spin to the right
    def spinRight(self):
        sim.setJointTargetVelocity(self.leftMotor, self.speed)
        sim.setJointTargetVelocity(self.rightMotor, -self.speed)

#############################################################################
if __name__ == '__main__':
    print('Program started')
    # connect to Remote API
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    #Getting Handles of the two Motors
    leftMotor = sim.getObject("/bubbleRob/leftMotor")
    rightMotor = sim.getObject("/bubbleRob/rightMotor")
    #print(leftMotor, rightMotor)

    # Create the Qt Application
    app = QApplication(sys.argv)
    # Create form
    form = Form(leftMotor, rightMotor)
    #Show Form
    form.show()
    # Run the main Qt loop
    app.exec()
    #Stop the Simulation
    sim.stopSimulation()
    print("Simulation stepped back")

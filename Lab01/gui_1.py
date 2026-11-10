#!/usr/bin/env python3
# coding: utf-8
import sys
#import os
#import time

#this is the module for our GUI
from PySide6.QtWidgets import (QLineEdit, QPushButton, QApplication,
    QVBoxLayout, QDialog, QGridLayout)

from coppeliasim_zmqremoteapi_client import *

#This is the class of our GUI. In the main we create an Instance of this class.
class Form(QDialog):

    def __init__(self, leftMotor, rightMotor, parent=None):
        
        super(Form, self).__init__(parent)

        # Initialize client, motors and speed
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.speed = 1.0

        # Create button objects
        self.buttonStart = QPushButton("Start")
        self.buttonStop = QPushButton("Stop")
        # Create the layout
        layout = QGridLayout()
        # Order the buttons in the layout first number is the row, second is the column
        layout.addWidget(self.buttonStart,4,1)
        layout.addWidget(self.buttonStop,5,1)
        # Set dialog layout
        self.setLayout(layout)

        # Add the methods to the buttons that should get executed when you press on them
        self.buttonStart.clicked.connect(self.startSim)
        self.buttonStop.clicked.connect(self.stopSim)

    #This method starts the simulation
    def startSim(self):
        status_code_start = sim.startSimulation()
        print("Simulation started")
        sim.setJointTargetVelocity(self.leftMotor, -self.speed)
        sim.setJointTargetVelocity(self.rightMotor, self.speed)

    #Stop Simulation
    def stopSim(self):
        #Stop the Simulation
        sim.setJointTargetVelocity(self.leftMotor, 0.0)
        sim.setJointTargetVelocity(self.rightMotor, 0.0)
        print("Simulation stopped")
###############################################################################################################################
if __name__ == '__main__':

    print('Program started')

    client = RemoteAPIClient()
    sim = client.getObject('sim')

    #Getting Handles of the two Motors
    leftMotor = sim.getObject("./leftMotor")
    rightMotor = sim.getObject("./rightMotor")
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

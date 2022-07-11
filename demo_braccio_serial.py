# This program demonstrates how to get the inverse kinematic solution for the braccio robotic arm and send it to the robot via serial communication.
# The program needs the arduino to be loaded with "serialBraccio.ino" example from the a Arduino library, "BraccioRobot" <https://github.com/stefangs/arduino-library-braccio-robot>

import geometrIK as gIK
from math import *
import numpy as np
from matplotlib import pyplot as plt
import serial
from serial.tools import list_ports
import time

ports = serial.tools.list_ports.comports()

for i, port in enumerate(ports):
    print(i, port[0])

port_choice = int(input("Enter port number: "))

s = serial.Serial()
s.port = str(ports[port_choice][0])
s.baudrate = 115200
s.parity = "N"
s.bytesize = 8
s.stopbits = 1
s.open()

time.sleep(2)
sendString = "P90,90,90,90,90,73,100"
print(sendString)
print(s.write(bytes(sendString, "utf-8")))
time.sleep(2)

robot = gIK.geometrIK()

while True:
    command = input("Enter command: ")
    if command == "quit":
        break
    x, y, z = (int(s) for s in input("Enter coordinates: ").split(","))
    solns = []
    robot.set_coordinates(x, y, z)
    solns = robot.solve(transform=True)
    if len(solns) == 0:
        print("No solution")
    else:
        for i, soln in enumerate(solns):
            print(i, soln)

        braccio_angles = solns[int(input("Enter soln number: "))]
        braccio_angles.append(100)
        print(braccio_angles)
        sendString = "P" + str(braccio_angles)[1:-1].replace(" ", "")
        print(sendString)

        print(s.write(bytes(sendString, "utf-8")))
        time.sleep(2)

sendString = "P90,90,90,90,90,73,100"
print(sendString)
print(s.write(bytes(sendString, "utf-8")))

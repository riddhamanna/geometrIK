# This program demonstrates how to get the inverse kinematic solution for a custom made robotic arm

import geometrIK as gIK
from math import *
import numpy as np
from matplotlib import pyplot as plt

# Initialise a robot object
robot = gIK.geometrIK()

# Define the wrist and gripper orientations
robot.wrist = 90  # degrees
robot.gripper = 73  # degrees

# Define the link lengths, starting from the base. The last length is equal to the length of the last link plus the length of the end effector
robot.links = [71.5, 125.0, 125.0, 192.0]  # mm

# Define the constraints for each joint angles
robot.constraints = [range(181), range(20, 161), range(181), range(181)]  # degrees

# Set the target coordinate of the end effector for which the joint angles need to be calculated
robot.set_coordinates(100, 80, 150)  # mm

# Get all possible solutions for a particular end effector coordinate. Set transform to True if you need the raw angles converted to your specific setup.
soln = robot.solve(transform=True)

# Show all possible joint angle solutions
print(*soln, sep="\n")

# Show the pose of the robot in all possible configurations to reach the end effector target coordinates
plt.show()

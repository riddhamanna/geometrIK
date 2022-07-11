# geometricIK
A small library to calculate inverse kinematics of 6 DOF robotic arms.

Joint 1 rotates around the z axis.
Joint 2, 3, 4 rotate around the y axis.
Joint 5 is the wrist orientation.
Joint 6 is controls the gripper.

Joint 5 and 6 needs to be set manually.

The link lengths (in mm), and the joint angle constraints (in degrees) for each joint can be set in terms of its operating range.

Either raw angles between the links or angles transformed in terms of servo rotations can be obtained.

All the defaults are set for the Arduino TinkerKit Braccio robotic arm.

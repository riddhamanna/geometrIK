from math import *
import numpy as np
from matplotlib import pyplot as plt


class geometrIK:
    def __init__(self):
        self.X = 100.0
        self.Y = 80.0
        self.Z = 150.0
        self.theta = 45
        self.wrist = 90
        self.gripper = 73
        self.links = [71.5, 125.0, 125.0, 192.0]
        self.link1 = self.links[0]
        self.link2 = self.links[1]
        self.link3 = self.links[2]
        self.link4 = self.links[3]
        self.constraints = [range(181), range(20, 161), range(181), range(181)]
        self.solutions = []

    def set_coordinates(self, X, Y, Z):
        """Set the target coordinates for the position of the end effector in real world coordinates with the origin at the robot base"""
        self.X = X
        self.Y = Y
        self.Z = Z

    def get_coordinates_joints(self):
        """Gets the coordinates of the joints given the coordinates of the end effector and an approach angle"""
        x = self.X
        y = self.Y
        z = self.Z - self.link1

        m1 = degrees(atan(y / x))

        a = sqrt(x**2 + y**2 + z**2)
        b = self.link4
        c = self.link3
        d = self.link2

        A = (0, 0)
        B = (a, 0)

        p = a - b * cos(radians(self.theta))
        q = b * sin(radians(self.theta))

        C = (p, q)

        temp = p * p + q * q + d * d - c * c
        a1 = 2 * p / temp
        b1 = 2 * q / temp
        m = -1 * a1 / b1
        c1 = 1 / b1
        Disc = 4 * m * m * c1 * c1 - 4 * (m * m + 1) * (c1 * c1 - d * d)

        # Works for majority of the cases
        p1 = (-2 * m * c1 - sqrt(Disc)) / (2 * (m * m + 1))

        # Uncomment the next line if no solution is reported. If it still fails, no real solution possible
        # p1 = (-2*m*c1 + sqrt(Disc))/(2*(m*m + 1))

        q1 = m * p1 + c1

        D = (p1, q1)

        return (A, B, C, D)

    def dist_between_2_points(self, A, B):
        """Calculates the absolute distance between two points given their coordinates. Not used in inverse kinematic calculations."""
        return sqrt((B[1] - A[1]) ** 2 + (B[0] - A[0]) ** 2)

    def get_angle_between_two_vectors(self, p1, p_common, p2):
        """Calculates the angle between two vectors with a common (corner) point. Positive in anti-clockwose direction"""
        v1 = np.array(p1) - np.array(p_common)
        v2 = np.array(p2) - np.array(p_common)
        angle = np.math.atan2(np.linalg.det([v1, v2]), np.dot(v1, v2))
        return np.degrees(angle)

    def get_angles(self, theta, transform=False):
        """Gets the joint angles for a given coordinate of the end effector and aapproach angle. Checks for joint angle constraints before reporting solution."""
        x = self.X
        y = self.Y
        z = self.Z - self.link1
        self.theta = theta
        l = sqrt(x**2 + y**2)

        A, B, C, D = self.get_coordinates_joints()

        m1 = round(degrees(atan(y / x)))
        m2_1 = degrees(atan(z / l))
        m2_2 = self.get_angle_between_two_vectors(B, A, D)
        m2 = round(m2_1 + m2_2)
        m3 = round(self.get_angle_between_two_vectors(A, D, C))
        m4 = round(self.get_angle_between_two_vectors(D, C, B))

        if transform:
            m1, m2, m3, m4 = self.transform_angles(m1,m2,m3,m4)
        m5 = self.wrist
        m6 = self.gripper

        if (
            m1 in self.constraints[0]
            and m2 in self.constraints[1]
            and m3 in self.constraints[2]
            and m4 in self.constraints[3]
        ):
            x_coord = np.array([A[0], D[0], C[0], B[0]])
            y_coord = np.array([A[1], D[1], C[1], B[1]])
            costheta = cos(-1 * radians(m2_1))
            sintheta = sin(-1 * radians(m2_1))
            tx = [0] + list(costheta * x_coord + sintheta * y_coord)
            ty = [0] + list(costheta * y_coord - sintheta * x_coord + self.link1)
            plt.plot(tx,ty)
            return (m1, m2, m3, m4, m5, m6)
        else:
            return None

    def transform_angles(self,m1,m2,m3,m4):
        """Converts raw angles to robot spesific joint angles. Customised for Arduino TinkerKit Braccio robotic arm, in a particular setup. Needs to be customised for your model and setup"""
        m1 = 180 - m1
        m2 = 180 - m2
        m3 = 360 - m3 - 90
        m4 = 360 - m4 - 90

        return(m1,m2,m3,m4)

    def solve(self,transform=False):
        """Reports and shows all posible orientations of the robot to reach the given end effector coordinates starting from an approach angle of 1 to 180 degrees"""
        for i in range(0, 181):
            try:
                m1, m2, m3, m4, m5, m6 = self.get_angles(i,transform)
                angles = [m1, m2, m3, m4, m5, m6]
                self.solutions.append(angles)

            except Exception as e:
                pass
        return self.solutions

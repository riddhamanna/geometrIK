# This program demonstrates how to get the inverse kinematic solution for the braccio robotic arm and send it to the robot via serial communication.
# The program needs the arduino to be loaded with "serialBraccio.ino" example from the a Arduino library, "BraccioRobot" <https://github.com/stefangs/arduino-library-braccio-robot>

import geometrIK as gIK
from math import *
import numpy as np
from matplotlib import pyplot as plt
import serial
from serial.tools import list_ports
import cv2
import time

#declarations and definitions for robot
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
current_braccio_angles = [90, 90, 90, 90, 90, 73, 100]

robot = gIK.geometrIK()

def send_command(command):
    global current_braccio_angles
    if command == "quit":
        print("bye bye")
    elif command == "open_gripper":
        current_braccio_angles[5] = 20
        robot.gripper = 20
        print(current_braccio_angles)
        sendString = "P" + str(current_braccio_angles)[1:-1].replace(" ", "")
        print(sendString)
        print(s.write(bytes(sendString, "utf-8")))
        time.sleep(2)
    elif command == "close_gripper":
        current_braccio_angles[5] = 73
        robot.gripper = 73
        print(current_braccio_angles)
        sendString = "P" + str(current_braccio_angles)[1:-1].replace(" ", "")
        print(sendString)
        print(s.write(bytes(sendString, "utf-8")))
        time.sleep(2)
    elif len(command.split(",")) == 3:
        x, y, z = (int(s) for s in command.split(","))
        solns = []
        robot.set_coordinates(x, y, z)
        solns = robot.solve(transform=True)
        if len(solns) == 0:
            print("No solution")
        else:
            braccio_angles = solns[0]
            braccio_angles.append(100)
            current_braccio_angles = braccio_angles
            print(braccio_angles)
            sendString = "P" + str(braccio_angles)[1:-1].replace(" ", "")
            print(sendString)
            print(s.write(bytes(sendString, "utf-8")))
            time.sleep(2)
    elif command == "home":
        current_braccio_angles[0] = 90
        current_braccio_angles[1] = 90
        current_braccio_angles[2] = 90
        current_braccio_angles[3] = 90
        current_braccio_angles[4] = 90
        print(current_braccio_angles)
        sendString = "P" + str(current_braccio_angles)[1:-1].replace(" ", "")
        print(sendString)
        print(s.write(bytes(sendString, "utf-8")))
        time.sleep(2)
    else:
        print("Inavlid command")

# declarations and definitions for aruco detection and percpective transform
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

arucoDictStr = "DICT_6X6_100"

topleftID = 1
toprightID = 2
bottomleftID = 3
bottomrightID = 4
objectID = 5 #6,7,8

width = 300
height = 200
factor = 1

trXY = [0,0,1]
getXY = [0,0,1]

objectX = 150
objectY = 100

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,4096)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,3000)
cap.set(cv2.CAP_PROP_FPS,30)

ptsSrc = [[0,0],[width*factor,0],[0,height*factor],[width*factor,height*factor]]
ptsDes = np.float32([[0,0],[width*factor,0],[0,height*factor],[width*factor,height*factor]])

cv2.namedWindow('image')

# movement instructions
deltaX = 80
deltaY = 20
def execute_move():
    braccioY = objectX + deltaY
    braccioX = objectY + deltaX
    send_command(command="home")
    time.sleep(1)
    send_command(command="open_gripper")
    time.sleep(1)
    send_command(command= str(braccioX)+","+str(braccioY)+",80")
    time.sleep(2)
    send_command(command="close_gripper")
    time.sleep(1)
    send_command(command="home")
    time.sleep(2)
    send_command(command="1,150,100")
    time.sleep(2)
    send_command(command="open_gripper")
    time.sleep(1)
    send_command(command="home")
    time.sleep(1)

while True:
    ret, image = cap.read()
    # image = cv2.imread('image.jpg')
    # cv2.imwrite("grid.png",img)
    h,w,layers = image.shape
    h = int(h/4)
    w = int(w/4)


    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[arucoDictStr])
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

    target_corners_found = 0
    object_found = False
    object_found_within_box = False

    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        ids = ids.flatten()
        target_corners_found = 0
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = [int(topRight[0]), int(topRight[1])]
            bottomRight = [int(bottomRight[0]), int(bottomRight[1])]
            bottomLeft = [int(bottomLeft[0]), int(bottomLeft[1])]
            topLeft = [int(topLeft[0]), int(topLeft[1])]
            # draw the bounding box of the ArUCo detection
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 0, 255), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 0, 255), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the image
            cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if markerID == topleftID:
                ptsSrc[0]=topLeft
                target_corners_found += 1
            if markerID == toprightID:
                ptsSrc[1]=topRight
                target_corners_found += 1
            if markerID == bottomleftID:
                ptsSrc[2]=bottomLeft
                target_corners_found += 1
            if markerID == bottomrightID:
                ptsSrc[3]=bottomRight
                target_corners_found += 1
            if markerID == objectID:
                objectX = cX
                objectY = cY
                object_found = True

    image_show = cv2.resize(image,(w,h))
    cv2.imshow("image",image_show)
    if target_corners_found == 4:
        cv2.namedWindow('transformed')
        ptsSrc = np.float32(ptsSrc)
        TrMat = cv2.getPerspectiveTransform(ptsSrc,ptsDes)
        transformed = cv2.warpPerspective(image,TrMat,(width*factor,height*factor))
        if object_found:
            trXY = [objectX*factor,objectY*factor,1]
            getXY = np.matmul(TrMat,trXY)
            getXY = getXY/getXY[2]
            objectX = int(getXY[0]/factor)
            objectY = int(getXY[1]/factor)
            if objectX >=0 and objectX <=300 and objectY >=0 and objectY <= 200:
                object_found_within_box = True
                cv2.putText(transformed, "X",(objectX, objectY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("transformed",transformed)

    key = cv2.waitKey(1)
    if key == ord('m') and object_found_within_box:
        key = ord('x')
        execute_move();
    if key == 27:#ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

sendString = "P90,90,90,90,90,73,100"
print(sendString)
print(s.write(bytes(sendString, "utf-8")))

import pickle
import numpy as np
import cv2
import math
from math import atan2, sin, cos, asin, acos, pi
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from itertools import product, combinations
i = 0
SumaX = 0
SumaY = 0
SumaZ = 0
Xcam = 0
Ycam = 0
Zcam = 0
params = pickle.load(open('ParametrosCamara.p', 'rb'))
mtx = params['mtx']
dist = params['dist']
valoresX = [0, 0]
valoresY = [0, 0]
valoresZ = [0, 0]
cap = cv2.VideoCapture(2)
fig = plt.figure()
ax = plt.axes(projection='3d')
plt.ion()

if not cap.isOpened():
    print('Cannot open camera')

while True:
    ret, frame = cap.read()
    h, w, c = frame.shape
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected_img_points) = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for (marker_corners, marker_id) in zip(corners, ids):
            corners = marker_corners.reshape((4, 2))
            (tl, tr, br, bl) = corners

            tl = (int(tl[0]), int(tl[1]))
            tr = (int(tr[0]), int(tr[1]))
            br = (int(br[0]), int(br[1]))
            bl = (int(bl[0]), int(bl[1]))

            cv2.line(frame, tl, tr, (0, 255, 0), 2)
            cv2.line(frame, tr, br, (0, 0, 255), 2)
            cv2.line(frame, br, bl, (0, 255, 255), 2)
            cv2.line(frame, bl, tl, (255, 0, 0), 2)

            cx = (tl[0] + br[0]) // 2
            cy = (tl[1] + br[1]) // 2

            cv2.circle(frame, (cx, cy), 10, (255, 0, 255), 2)

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, 0.025, mtx, dist)
            (rvec - tvec).any()

            x = tvec[0][0][0] * 100
            y = tvec[0][0][1] * 100
            z = tvec[0][0][2] * 100

            phi = rvec[0][0][0]
            theta = rvec[0][0][1]
            psi = rvec[0][0][2]

            cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.05)
            Rx = np.array([[1, 0, 0, 0],
                           [0, cos(phi), -sin(phi), 0],
                           [0, sin(phi), cos(phi), 0],
                           [0, 0, 0, 1]])

            Ry = np.array([[cos(theta), 0, sin(theta), 0],
                           [0, 1, 0, 0],
                           [-sin(theta), 0, cos(theta), 0],
                           [0, 0, 0, 1]])

            Rz = np.array([[cos(psi), -sin(psi), 0, 0],
                           [sin(psi), cos(psi), 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

            R = Rx @ Ry @ Rz

            MatrizR = R[:3, :3]

            VectorT = np.array([[x],
                                [y],
                                [z]])

            R_inv = MatrizR.T
            t_inv = -R_inv @ VectorT

            MatrizCamara = np.identity(4)
            MatrizCamara[:3, :3] = R_inv
            MatrizCamara[:3, 3] = t_inv[:3, 0]

            if len(ids) > 1:
                valoresX[i] = t_inv[0, 0]
                valoresY[i] = t_inv[1, 0]
                valoresZ[i] = t_inv[2, 0]
                if i < 1:
                    i = i + 1
                else:
                    for n in valoresX:
                        SumaX = SumaX + n

                    for n in valoresY:
                        SumaY = SumaY + n

                    for n in valoresZ:
                        SumaZ = SumaZ + n

                    Xcam = SumaX / len(ids)
                    Ycam = SumaY / len(ids)
                    Zcam = SumaZ / len(ids)
                    print(Xcam)
                    print(Ycam)
                    print(Zcam)
                    print('----')
                    i = 0

            else:
                Xcam = t_inv[0, 0]
                Ycam = t_inv[1, 0]
                Zcam = t_inv[2, 0]
                print(Xcam)
                print(Ycam)
                print(Zcam)
                print('----')

            axes = np.array([[5, 0, 0, 0, 0],
                             [0, 0, 5, 0, 0],
                             [0, 0, 0, 0, 5],
                             [1, 1, 1, 1, 1]])

            vectors = MatrizCamara @ axes

            plt.cla()
            ax.set_xlim(-50, 50)
            ax.set_ylim(-50, 50)
            ax.set_zlim(-10, 50)
            ax.plot3D(Xcam, Ycam, Zcam, marker='o', markersize=5)
            ax.set_title('3D Aruco')

            x = [-5, 5]
            y = [-5, 5]
            z = [0, 0]

            ax.plot3D(vectors[0, :], vectors[1, :], vectors[2, :], marker='o', markersize=1)
            SumaX = 0
            SumaY = 0
            SumaZ = 0
            for s, e in combinations(np.array(list(product(x, y, z))), 2):
                if np.sum(np.abs(s - e)) == x[1] - x[0]:
                    ax.plot3D(*zip(s, e), color="b")
            plt.draw()
            plt.pause(0.001)

    cv2.imshow('camera', frame)
    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()
cap.release()

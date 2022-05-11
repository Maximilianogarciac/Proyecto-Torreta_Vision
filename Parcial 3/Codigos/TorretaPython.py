import pickle
import numpy as np
import cv2
import matplotlib.pyplot as plt
import serial
import time
xx = []
yy = []
tt = []
fig = plt.figure()
plt.ion()
start = time.time()
TotalEx, TotalEy = 0, 0
ErrorX, ErrorY = 0, 0
serialArduino = serial.Serial("COM5", 9600, timeout=1)
params = pickle.load(open('ParametrosCamara.p', 'rb'))
mtx = params['mtx']
dist = params['dist']
Posicion = [0, 60, 120, 180]
SC, SB = 0, 0
kx, ky = 0.0295, 0.0195
kix, kiy = 0.0024, 0.0024
i, j, l = 0, 0, 0
tol = 10
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print('Cannot open camera')

while True:
    ret, frame = cap.read()
    h, w, c = frame.shape
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected_img_points) = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    kprima, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    frametwo = cv2.undistort(frame, mtx, dist, None, kprima)
    frametwo = frametwo[roi[1]:roi[1] + roi[3], roi[0]: roi[0] + roi[2]]

    if i >= 10:

        if l == 0:
            serialArduino.write(bytes('{},{},{};'.format(180, -180, 0), 'ascii'))
            serialArduino.write(bytes('{},{},{};'.format(-90, 0, 0), 'ascii'))
            l = l +1

        if j > 180:
            serialArduino.write(bytes('{},{},{};'.format(0, -180, 0), 'ascii'))
            j = 0
        else:
            j = j + 1
            serialArduino.write(bytes('{},{},{};'.format(0, 2, 0), 'ascii'))
            cv2.waitKey(10)
    else:
        i = i + 1

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
            fx = mtx[0, 0]
            fy = mtx[1, 1]
            cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.05)
            rx = w / 2 + ((fx/1000) * 6)
            ry = h / 2 + ((fy/1000) * -28)
            cv2.circle(frame, (int(rx), int(ry)), 10, (0, 255, 0), 2)
            ErrorX = (cx - rx)
            ErrorY = (cy - ry)
            TotalEx = (TotalEx + ErrorX)
            TotalEy = (TotalEy + ErrorY)
            norma = np.sqrt(ErrorX**2 + ErrorY**2)

            if norma > tol:
                SB = ((ErrorX * kx) + TotalEx * kix) * -1
                SC = ((ErrorY * ky) + TotalEy * kiy) * 1
                L = 0
            else:
                SB = 0
                SC = 0
                L = 1

            serialArduino.write(bytes('{},{},{};'.format(SC, SB, L), 'ascii'))
            i = 0
            l = 0
            j = 0

    now = time.time()
    t = now - start
    tt.append(t)
    xx.append(ErrorX)
    yy.append(ErrorY)
    plt.cla()
    plt.plot(tt, xx, label='Error en el eje X')
    plt.plot(tt, yy, label = 'Error en el eje Y')
    plt.xlabel('Tiempo (Segundos)')
    plt.ylabel('Error (Pixeles)')
    plt.legend()
    plt.grid(True)
    plt.draw()
    plt.pause(0.01)
    cv2.imshow('camera', frame)

    key = cv2.waitKey(1)
    if key == 27:
        cv2.imwrite('Imagen.png', frame)
        plt.savefig("Grafica.pdf", dpi=150)
        break
serialArduino.close()
cv2.destroyAllWindows()
cap.release()
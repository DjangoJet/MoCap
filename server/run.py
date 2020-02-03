import socket
import select
import json
import sys
import pickle
import cv2
import numpy as np
import math
import threading
import concurrent.futures

import server

class Points3D:
    def __init__(self, M_L, d_L, M_R, d_R, P_L, P_R, F):
        self.markersNumber = 4
        self.F = F
        self.M_L = M_L
        self.d_L = d_L
        self.M_R = M_R
        self.d_R = d_R        
        self.P_R = P_R
        self.P_L = P_L
        self.takePeaceParameters()

    def takePeaceParameters(self):
        self.leftK1 = self.d_L[0, 0]
        self.leftK2 = self.d_L[0, 1]
        self.leftP2 = self.d_L[0, 2]
        self.leftP1 = self.d_L[0, 3]
        self.leftK3 = self.d_L[0, 4]
        
        self.rightK1 = self.d_R[0, 0]
        self.rightK2 = self.d_R[0, 1]
        self.rightP1 = self.d_R[0, 2]
        self.rightP2 = self.d_R[0, 3]
        self.rightK3 = self.d_R[0, 4]
        
        self.rightCx = self.M_R[0,2]
        self.rightCy = self.M_R[1,2]
        self.rightFx = self.M_R[0,0]
        self.rightFy = self.M_R[1,1]
        
        self.leftCx = self.M_L[0,2]
        self.leftCy = self.M_L[1,2]
        self.leftFx = self.M_L[0,0]
        self.leftFy = self.M_L[1,1]

    def undisortPoints(self, pointsLeft, pointsRight):
        try:
            flagRightOne = False
            flagLeftOne = False
            if len(pointsRight) == 1:
                pointsRight.append(pointsRight[0])
                flagRightOne = True
            pointsRight = np.asarray(pointsRight) # 3x2
            pointsRight = np.expand_dims(pointsRight.T, axis=1) # 2x1x3
            pointsRight = cv2.undistortPoints(pointsRight.T, self.M_R, self.d_R) # 3x1x2 TU JEST BŁĄD DLA JEDNEGO ELEMENTU
            # add undisort for one point
            pointsRight = pointsRight[:,0,:] # 3x2
            pointsRight[:,0] = pointsRight[:,0]*self.rightFx + self.rightCx # 3x2
            pointsRight[:,1] = pointsRight[:,1]*self.rightFy + self.rightCy # 3x2

            if len(pointsLeft) == 1:
                pointsLeft.append(pointsLeft[0])
                flagLeftOne = True
            pointsLeft = np.asarray(pointsLeft)
            pointsLeft = np.expand_dims(pointsLeft.T, axis=1)
            pointsLeft = cv2.undistortPoints(pointsLeft.T, self.M_L, self.d_L)
            # add undisort for one point
            pointsLeft = pointsLeft[:,0,:]
            pointsLeft[:,0] = pointsLeft[:,0]*self.leftFx + self.leftCx
            pointsLeft[:,1] = pointsLeft[:,1]*self.leftFy + self.leftCy

            if flagRightOne:
                pointsRight = pointsRight[0,:]
                pointsRight = pointsRight.reshape((2,1)).T
            if flagLeftOne:
                pointsLeft = pointsLeft[0,:]
                pointsLeft = pointsLeft.reshape((2,1)).T
            return pointsLeft, pointsRight # 3x2, 3x2
        except:
            print('Error in undisortingPoints()')
            sys.exit()
        
    
    def paringPoints(self, pointsLeft, pointsRight):
        try:
            paringPointsLeft = []
            paringPointsRight = []
            minQuantity = 1
            pointsLeft = cv2.convertPointsToHomogeneous(np.asarray(pointsLeft))
            pointsRight = cv2.convertPointsToHomogeneous(np.asarray(pointsRight))
            findPoint = False
            if len(pointsLeft) >= len(pointsRight):          
                for pointL in pointsLeft[:,0,:]:
                    tempBestPointRight = np.zeros((3, 1))
                    tempBestPointLeft = np.zeros((3, 1))
                    for pointR in pointsRight[:,0,:]:
                        tempQuantity = self.takeParingQuantityIndicator(pointL, pointR)
                        if tempQuantity < minQuantity:
                            findPoint = True
                            tempBestPointLeft = pointL
                            tempBestPointRight = pointR
                            minQuantity = tempQuantity
                    if findPoint:
                        paringPointsLeft.append(tempBestPointLeft)
                        paringPointsRight.append(tempBestPointRight)
                    minQuantity = 1
                    findPoint = False

                if len(paringPointsLeft) > 0:
                    paringPointsLeft = np.asarray(paringPointsLeft)
                    paringPointsLeft = paringPointsLeft[:,:2]

                if len(paringPointsRight) > 0:
                    paringPointsRight = np.asarray(paringPointsRight)
                    paringPointsRight = paringPointsRight[:,:2]
                return paringPointsLeft, paringPointsRight
            else:
                for pointR in pointsRight[:,0,:]:
                    tempBestPointRight = np.zeros((3, 1))
                    tempBestPointLeft = np.zeros((3, 1))
                    for pointL in pointsLeft[:,0,:]:
                        tempQuantity = self.takeParingQuantityIndicator(pointL, pointR)
                        if tempQuantity < minQuantity:
                            findPoint = True
                            tempBestPointLeft = pointL
                            tempBestPointRight = pointR
                            minQuantity = tempQuantity
                    if findPoint:
                        paringPointsLeft.append(tempBestPointLeft)
                        paringPointsRight.append(tempBestPointRight)
                    minQuantity = 1
                    findPoint = False

                if len(paringPointsLeft) > 0:
                    paringPointsLeft = np.asarray(paringPointsLeft)
                    paringPointsLeft = paringPointsLeft[:,:2]

                if len(paringPointsRight) > 0:
                    paringPointsRight = np.asarray(paringPointsRight)
                    paringPointsRight = paringPointsRight[:,:2]
                return paringPointsLeft, paringPointsRight
        except:
            print('Error in paringPoints()')
            sys.exit()
        
    def takeParingQuantityIndicator(self, pointL, pointR):
        return math.fabs(pointR.reshape((3,1)).T @ self.F @ pointL.reshape((3,1)))
    
    def triangulatePoints(self, pointsLeft, pointsRight):
        points3D = cv2.triangulatePoints(self.P_R, self.P_L, pointsRight.T, pointsLeft.T)
        points3D = cv2.convertPointsFromHomogeneous(points3D.T)
        return points3D

def uploadCamerasParameters():
    with open ('/home/mike/Code/Mocap/server/params', 'rb') as f:
        params = pickle.load(f)
        M_R = params["M_R"]
        M_L = params["M_L"]
        d_R = params["d_R"]
        d_L = params["d_L"]
        F = params["F"]
        P_L = params["P_L"]
        P_R = params["P_R"] 
        points3DMaker = Points3D(M_L, d_L, M_R, d_R, P_L, P_R, F)
        return points3DMaker

def points3Dcalculation(cameraLeftPoint, cameraRightPoint, points3DMaker):
    if len(cameraLeftPoint) > 0 and len(cameraRightPoint) > 0:
        cameraLeftPoint, cameraRightPoint = points3DMaker.undisortPoints(cameraLeftPoint, cameraRightPoint)
        cameraLeftPoint, cameraRightPoint = points3DMaker.paringPoints(cameraLeftPoint, cameraRightPoint)

        if len(cameraLeftPoint) > 0 and len(cameraRightPoint) > 0:
            if len(cameraLeftPoint) <= points3DMaker.markersNumber:
                points3D = points3DMaker.triangulatePoints(cameraLeftPoint, cameraRightPoint)
                return points3D
            else:
                return 'Detected points are bigger than markers number'
        else:
            return 'Some paring lists points are empty'
    else:
        return 'Some points from cameras are empty'

if __name__ == "__main__":
    points3DMaker = uploadCamerasParameters()
    HOST, PORT = "192.168.43.60", 5000
    pointServer = server.ThreadedTCPServer((HOST, PORT), server.Server)
    t = threading.Thread(target=pointServer.serve_forever)
    t.setDaemon(True)
    t.start()
    try:
        while True:
            if server.flagCameraLeft and server.flagCameraRight:
                # print(server.cameraLeftPoint)
                # print(server.cameraRightPoint)
                # print('\n')
                # server.camera3DPoints = ['I have both']
                server.flagSendPoints = True
                server.flagCameraLeft = False
                server.flagCameraRight = False
                with concurrent.futures.ProcessPoolExecutor() as executor:
                    process = executor.submit(points3Dcalculation, server.cameraLeftPoint, server.cameraRightPoint, points3DMaker)
                    output = process.result()
                    if type(output) == str:
                        print(output)
                    else:
                        print(output)
                        print(type(output))
                        print('\n')
                        server.camera3DPoints = output
    except:
        sys.exit()
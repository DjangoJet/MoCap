import socket
import select
import json
import sys
import pickle
import cv2
import numpy as np
import math
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
        
    
    def paringPoints(self, pointsLeft, pointsRight):
        paringPointsLeft = []
        paringPointsRight = []
        minQuantity = 1
        pointsLeft = cv2.convertPointsToHomogeneous(pointsLeft)
        pointsRight = cv2.convertPointsToHomogeneous(pointsRight)
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
        
    def takeParingQuantityIndicator(self, pointL, pointR):
        return math.fabs(pointR.reshape((3,1)).T @ self.F @ pointL.reshape((3,1)))
    
    def triangulatePoints(self, pointsLeft, pointsRight):
        points3D = cv2.triangulatePoints(self.P_R, self.P_L, pointsRight.T, pointsLeft.T)
        points3D = cv2.convertPointsFromHomogeneous(points3D.T)
        return points3D

class Server:
    def __init__(self, host, port, cameraNumber):
        self.host = host
        self.port = port
        self.cameraNumber = cameraNumber
        self.setupServer()
    
    def setupServer(self):
        self.socketServer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socketServer.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socketServer.bind((self.host, self.port))
        self.socketServer.listen()
        print(f"Main server started on port {self.port}")

    def connectCameras(self):
        self.connectionList = []
        while len(self.connectionList) != self.cameraNumber:
            sockfd, addr = self.socketServer.accept()
            self.connectionList.append(sockfd)
            print("Client (%s, %s) connected" % addr)

        print(f"{self.cameraNumber} cameras are connected")

    def identifyCameras(self):
        self.params = []
        for sock in self.connectionList:
            self.params.append(pickle.loads(sock.recv(4096)))

    def sendOk(self):
        for sock in self.connectionList:
            sock.sendall(b'ok')

    def startCollectingPoints(self, points3DMaker):
        self.sendOk()
        while True:
            read_sockets, _, _ = select.select(self.connectionList,[],[])
            if len(read_sockets) == self.cameraNumber:
                pointsLeftGlob = []
                pointsRightGlob = []
                for index, sock in enumerate(self.connectionList):
                    data = pickle.loads(sock.recv(4096))
                    if self.params[index] == 'left':
                        pointsLeftGlob = data
                    elif self.params[index] == 'right':
                        pointsRightGlob = data
                self.sendOk()
                if len(pointsLeftGlob) > 0 and len(pointsRightGlob) > 0:
                    pointsLeftGlob, pointsRightGlob = points3DMaker.undisortPoints(pointsLeftGlob, pointsRightGlob) # 3x2
                    pointsLeftGlob, pointsRightGlob = points3DMaker.paringPoints(pointsLeftGlob, pointsRightGlob) # 3x2

                    if len(pointsLeftGlob) > 0 and len(pointsRightGlob) > 0:
                        if len(pointsLeftGlob) > 0 and len(pointsRightGlob) > 0:
                            if len(pointsLeftGlob) <= points3DMaker.markersNumber:
                                points3DGlob = points3DMaker.triangulatePoints(pointsLeftGlob, pointsRightGlob) # 3x3
                                print(points3DGlob)

                        else:
                            print('pointsLeft != pointsRight')
                    else:
                        print('Some paring points are empty')
                else:
                    print('pointsLeft or pointsRight ale empty')


def uploadCamerasParameters():
    with open ('/home/mike/Code/mocap/server/params', 'rb') as f:
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


if __name__ == "__main__":
    points3DMaker = uploadCamerasParameters()
    server = Server('192.168.43.60', 5000, 2)
    server.connectCameras()
    server.sendOk()
    server.identifyCameras()
    server.startCollectingPoints(points3DMaker)
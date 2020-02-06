# Import socket module 
import socket
import time
import pickle
import numpy as np
import math
import threading
import sys

data = []
flagData = False

def convertListToArray(points):
    points = np.asarray(points).T
    return points
  
def calculatePointsCenter(points):
    pointsCenter = sum(points, axis=1) / points.shape[1]
    return pointsCenter.reshape((3, 1))

def calculateVector(points, centerPoint):
    for i in range(points.shape[1] - 1):
        points[:, i] = points[:, i].reshape((3, 1)) - centerPoint
    return points

def matchingPoints():
    pass

def calculateRotationMatrix(points, basicPoints):
    a = points * basicPoints.T
    u, _, v = np.linalg.svd(a)
    return u * v.T

def calculateAngles(rotationMatrix):
    angleX = math.atan2(rotationMatrix[2, 1], rotationMatrix[2, 2])
    angleY = math.atan2(-rotationMatrix[2, 0], math.sqrt(rotationMatrix[2, 1]**2 + rotationMatrix[2, 2]**2))
    angleZ = math.atan2(rotationMatrix[1, 0]**2, rotationMatrix[0, 0])
    return angleX, angleY, angleZ

def client():
    host = '192.168.43.60'
    port = 5000
    global data
    global flagData
    try:
        s = socket.socket(socket.AF_INET,socket.SOCK_STREAM) 
        s.connect((host,port))
        name = 'client'
        s.send(pickle.dumps(name))
        while True:
            data = pickle.loads(s.recv(1024))
            flagData = True
    except:
        print('Error client')
        s.close()
        sys.exit()

if __name__ == '__main__': 
    global flagData
    global data

    t = threading.Thread(target=client)
    t.setDaemon(True)
    t.start()
    while True:
        msg = input('> ')
        if msg == 'point':
            if flagData == True:
                with open('point', 'wb') as f:
                    pickle.dump(data, f)
                print(data)
                msg = ''
            else:
                print('Unable to fetch point')
                msg = ''
        
        if msg == 'points':
            collectingTime = input('Give collecting time: ')
            collectingTime = float(collectingTime)
            collectingPoints = []
            startTime = time.time()
            while collectingTime > (time.time() - startTime):
                if flagData == True:
                    if len(data) == 3:
                        collectingPoints.append([data, time.time() - startTime])
                    else:
                        print('Marker points are not 3')
                    flagData = False
            
            print('Finish collecting points')
            with open('points','wb') as f:
                pickle.dump(collectingPoints, f)
            print('Points are saved in file')
            
            msg = ''
                
            
        
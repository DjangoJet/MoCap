import pickle
import numpy as np
import matplotlib.pyplot as plt
import math

def convertListToArray(points):
    points = np.asarray(points).T
    return points.reshape(points.shape[0], points.shape[2])
  
def calculatePointsCenter(points):
    pointsCenter = np.sum(points, axis=1) / points.shape[0]
    return pointsCenter.reshape((3, 1))

def calculateVectors(points, centerPoint):
    for i in range(points.shape[1]):
        points[:, i] = points[:, i] - centerPoint.reshape((3,))
    return points

def calculateRotationMatrix(pointsVector, basicPointsVector):
    a = pointsVector * basicPointsVector.T
    u, _, v = np.linalg.svd(a)
    return u * v.T

def calculateAngles(rotationMatrix):
    angleX = math.atan2(rotationMatrix[2, 1], rotationMatrix[2, 2])
    angleY = math.atan2(-rotationMatrix[2, 0], math.sqrt(rotationMatrix[2, 1]**2 + rotationMatrix[2, 2]**2))
    angleZ = math.atan2(rotationMatrix[1, 0]**2, rotationMatrix[0, 0])
    return angleX*180/np.pi, angleY*180/np.pi, angleZ*180/np.pi

def convertSymulationPoints(symPoints, basicPointsVector):
    tempSymPoints = []
    startSymulationTime = 0
    for i in range(len(symPoints)):
        if i == 0:
            startSymulationTime = symPoints[i][1]
            tempPoints = convertListToArray(symPoints[i][0])
            centerPoint = calculatePointsCenter(tempPoints)
            tempVectors = calculateVectors(tempPoints, centerPoint)
            tempRotationMatrix = calculateRotationMatrix(tempVectors, basicPointsVector)
            tempAngleX, tempAngleY, tempAngleZ = calculateAngles(tempRotationMatrix)
            tempSymPoints.append([centerPoint, np.array([[tempAngleX],[tempAngleY],[tempAngleZ]]), 0])
        else:
            tempPoints = convertListToArray(symPoints[i][0])
            centerPoint = calculatePointsCenter(tempPoints)
            tempVectors = calculateVectors(tempPoints, centerPoint)
            tempRotationMatrix = calculateRotationMatrix(tempVectors, basicPointsVector)
            tempAngleX, tempAngleY, tempAngleZ = calculateAngles(tempRotationMatrix)
            tempTime = symPoints[i][1] - startSymulationTime
            tempSymPoints.append([centerPoint, np.array([[tempAngleX],[tempAngleY],[tempAngleZ]]), tempTime])
    return tempSymPoints
    
if __name__ == "__main__":
    basicPoints = []
    with open('../client/point', 'rb') as f:
        basicPoints = pickle.load(f)
    basicPointsArray = convertListToArray(basicPoints)
    basicPointsCenter = calculatePointsCenter(basicPointsArray)
    basicPointsVector = calculateVectors(basicPointsArray, basicPointsCenter)

    symulationPoints = []
    with open('../client/points', 'rb') as f:
        symulationPoints = pickle.load(f)
    symulationPositions = convertSymulationPoints(symulationPoints, basicPointsVector)
    with open('symulationPoints', 'wb') as f:
        pickle.dump(symulationPositions, f)
import picamera
from picamera.array import PiRGBArray
import numpy as np
import time
import cv2
import json
import socket
import sys
import pickle

class Markers:
    def __init__(self):
        self.kernel = np.ones((3, 3), np.uint8)
        self.takeMarkerParams()

    def takeMarkerParams(self):
        with open('setting.json', 'r') as settingJson:
            setting = json.load(settingJson)
            self.minThresh = setting["minThresh"]
            self.maxThresh = setting["maxThresh"]
            self.resolution = setting["resolution"]
            self.framerate = setting["framerate"]

    def prepareImage(self, img):
        _, img = cv2.threshold(img[:,:,0], self.minThresh, self.maxThresh, cv2.THRESH_BINARY)
        img = cv2.dilate(img, self.kernel, iterations=1)
        return img

    def takeCenters(self, img):
        points = []
        _, contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            M = cv2.moments(c)
            points.append([M['m10'] / M['m00'], M['m01'] / M['m00']])
        return points

    def findMarkesPosition(self, img):
        img = self.prepareImage(img)
        return self.takeCenters(img)

class Client:
    def __init__(self):
        self.takeServerParams()
    
    def takeServerParams(self):
        with open('setting.json', 'r') as settingJson:
            setting = json.load(settingJson)
            self.host = setting["host"]
            self.port = setting["port"]

    def setupClient(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client.connect((self.host, self.port))
        except:
            print("Unable to connect")
            sys.exit()
        print("Connect to server")

    def receiveOk(self):
        message = ""
        while message != "ok":
            message = self.client.recv(1024).decode()
            print(type(message))
            print(message)
            if message != "ok":
                print("Message isn't ok")

    def sendMarkers(self, points):
        self.receiveOk()
        self.client.sendall(pickle.dumps(points))
        print("Send markers")

    def sendParameters(self):
        self.receiveOk()
        self.projectionCamera = np.load('projectionMatrixRight.npy')
        self.client.sendall(pickle.dumps(self.projectionCamera))
        print("Send projection Camera matrix")

    def startClient(self):
        markers = Markers()
        self.cameraSetup(markers)

        print("Start detect markers")
        while True:
            for capture in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                frame = capture.array
                print("Corect frame capture")
                self.sendMarkers(markers.findMarkesPosition(frame))
                self.rawCapture.truncate(0)
    
    def cameraSetup(self, markers):
        self.camera = picamera.PiCamera()
        self.camera.resolution = (markers.resolution[0], markers.resolution[1])
        self.camera.color_effects = (128, 128)
        self.camera.framerate = markers.framerate
        self.rawCapture = PiRGBArray(self.camera)
        time.sleep(2)

if __name__ == "__main__":
    client = Client()
    client.setupClient()
    client.sendParameters()
    client.startClient()
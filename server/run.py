import socket
import select
import json
import sys
import pickle
import cv2
import numpy as np
import math
import threading

from points import Points3D
import server

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

if __name__ == "__main__":
    points3DMaker = uploadCamerasParameters()
    HOST, PORT = "192.168.43.60", 5000
    pointServer = server.ThreadedTCPServer((HOST, PORT), server.Server)
    t = threading.Thread(target=pointServer.serve_forever)
    t.setDaemon(True)
    t.start()
    while True:
        if server.flagCameraLeft and server.flagCameraRight:
            print(server.cameraLeftPoint)
            print(server.cameraRightPoint)
            print('\n')
            server.camera3DPoints = ['I have both']
            server.flagSendPoints = True
            server.flagCameraLeft = False
            server.flagCameraRight = False

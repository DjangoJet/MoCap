import socket
import select
import json
import sys
import pickle
import cv2
import numpy as np

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

    def takeParametersFromCameras(self):
        self.params = []
        for sock in self.connectionList:
            self.params.append(pickle.loads(sock.recv(4096)))

    def sendOk(self):
        for sock in self.connectionList:
            sock.sendall(b'ok')
        print("Send first ok")

    def startCollectingPoints(self):
        self.sendOk()
        while True:
            read_sockets, _, _ = select.select(self.connectionList,[],[])
            if len(read_sockets) == self.cameraNumber:
                for sock in self.connectionList:
                    data = pickle.loads(sock.recv(4096))
                    data = np.asanyarray(data)
                    print(data)
                self.sendOk()
                # Do someting with markers from camers

if __name__ == "__main__":
    server = Server('192.168.43.60', 5000, 2)
    server.connectCameras()
    server.sendOk()
    server.takeParametersFromCameras()
    print(type(server.params[0]))
    print(server.params[0])
    print(type(server.params[1]))
    print(server.params[1])
    server.startCollectingPoints()
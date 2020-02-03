import socketserver
from threading import Thread
import pickle

cameraRightPoint = []
cameraLeftPoint = []
camera3DPoints = []
flagCameraLeft = False
flagCameraRight = False
flagSendPoints = False

class Server(socketserver.BaseRequestHandler):

    def setup(self):
        data = pickle.loads(self.request.recv(1024))
        if data == 'cameraLeft':
            self.nodeType = 'cameraLeft'
        elif data == 'cameraRight':
            self.nodeType = 'cameraRight'
        elif data == 'client':
            self.nodeType = 'client'

    def handle(self):
        if self.nodeType == 'cameraLeft':
            self.cameraLeft()
        elif self.nodeType == 'cameraRight':
            self.cameraRight()
        elif self.nodeType == 'client':
            self.client()
    
    def cameraRight(self):
        global cameraRightPoint
        global flagCameraRight
        self.request.sendall(b'ok')
        while True:
            if flagCameraRight == False:
                cameraRightPoint = pickle.loads(self.request.recv(1024))
                self.request.sendall(b'ok')
                flagCameraRight = True

    def cameraLeft(self):
        global cameraLeftPoint
        global flagCameraLeft
        self.request.sendall(b'ok')
        while True:
            if flagCameraLeft == False:
                cameraLeftPoint = pickle.loads(self.request.recv(1024))
                self.request.sendall(b'ok')
                flagCameraLeft = True

    def client(self):
        global camera3DPoints
        global flagSendPoints
        while True:
            if flagSendPoints == True:
                self.request.sendall(pickle.dumps(camera3DPoints))
                flagSendPoints = False


class ThreadedTCPServer(socketserver.ThreadingTCPServer, socketserver.TCPServer):
    pass
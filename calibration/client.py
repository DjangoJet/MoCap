import picamera
import socket
import sys

host = '192.168.43.60'
port = 5000

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.connect((host, port))

camera = picamera.PiCamera()
camera.resolution = (1296, 972) # (640, 480)
camera.start_preview()

imgCounter = 0
while True:
    message = socket.recv(1024).decode()
    if message == 'p':
        camera.capture('img/img_%d.jpg' % imgCounter)
        imgCounter += 1
    elif message == 'exit':
        camera.stop_preview()
        sys.exit()
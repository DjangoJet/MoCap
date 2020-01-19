import socket
import select
import json
import sys
import pickle

def connectCameras():
    pass

def takeParametersFromCameras():
    pass

def startCollectingPoints():
    pass

if __name__ == "__main__":

    CONNECTION_LIST = []
    RECV_BUFFER = 4096
    PORT = 5000
    CAMERA_NUMBER = 2

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('192.168.43.60', PORT))
    server_socket.listen()

    print(f"Main server started on port {PORT}")

    while len(CONNECTION_LIST) != CAMERA_NUMBER:
        sockfd, addr = server_socket.accept()
        CONNECTION_LIST.append(sockfd)
        print("Client (%s, %s) connected" % addr)
    
    print("Two camera in CONNECTION_LIST")
    print(CONNECTION_LIST)
    for sock in CONNECTION_LIST:
        sock.sendall(b'ok')
    print("Send first ok")

    while True:
        read_sockets, write_sockets, error_sockets = select.select(CONNECTION_LIST,[],[])
        if len(read_sockets) == CAMERA_NUMBER:
            for sock in read_sockets:
                data = pickle.loads(sock.recv(RECV_BUFFER))
                sock.sendall(b'ok')
            # Do someting with markers from camers
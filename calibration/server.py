import socket
import sys
import select

CONNECTION_LIST = []
PORT = 5000
HOST = '192.168.43.60'
CAMERA_NUMBER = 2

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))
server_socket.listen()

print(f'Wait for connect {CAMERA_NUMBER} cameras')
while len(CONNECTION_LIST) != CAMERA_NUMBER:
    sockfd, addr = server_socket.accept()
    CONNECTION_LIST.append(sockfd)
    print("Client (%s, %s) connected" % addr)

print(f'All cameras connected')
while True:
    message = input('New command: ')
    if message == 'exit':
        for sock in CONNECTION_LIST:
            sock.sendall(b'exit')
        server_socket.close()
        sys.exit()
    else:
        for sock in CONNECTION_LIST:
            sock.sendall(message.encode())
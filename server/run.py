import socket
import json
import sys
import pickle

if __name__ == "__main__":
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:   
        s.bind(('192.168.43.60', 5000))
        s.listen()
        connection, address = s.accept()
        with connection:
            print(f'Connected by {address}')
            while True:
                input()
                connection.sendall(b'ok')
                data = pickle.loads(connection.recv(4096))
                print("Take data points")
                print(data)

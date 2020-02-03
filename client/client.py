# Import socket module 
import socket
import time
import pickle
  
if __name__ == '__main__': 
    host = '192.168.43.60'
    port = 5000
    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM) 
    s.connect((host,port))
    name = 'client'
    s.send(pickle.dumps(name))
    while True:
        data = pickle.loads(s.recv(1024))
        print(data)
        print('\n')

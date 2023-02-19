import socket
import cv2
import pickle
import struct
import math
import time
import os
import glob

# Client socket
# create an INET, STREAMing socket :
client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
# host_ip = '<localhost>'# Standard loopback interface address (localhost)
host_ip = '192.168.1.95' # put the ip of the server here
port = 10051 # Port to listen on (non-privileged ports are > 1023)
# now connect to the web server on the specified port number
client_socket.connect((host_ip,port))
# client_socket.settimeout(5)

try:
    while True:
        # client_socket.send(b"Hi from client")
        msg = client_socket.recv(1024).decode("utf-8")
        print(msg)
        time.sleep(5)
except Exception as e:
    print(e)
    print("Client socket closed")
    client_socket.close()
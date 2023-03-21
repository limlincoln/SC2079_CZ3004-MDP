import socket
import threading
from queue import Queue
import cv2
import pickle
import struct
import time
import io
import zlib
#from signal import signal, SIGPIPE, SIG_DFL
import picamera
import numpy as np

from config import *


# Command
# cd /home/RP03/Desktop/Python\ Test/SC2079_CZ3004-MDP/RPI/Multithreading
# python server_main_thread.py 

#signal(SIGPIPE,SIG_DFL)
# Start server
class Connect_PC_Client(threading.Thread):

    def __init__(self, name, img_results, img_valid, shoot_signal, setup_info, car_path):
        super(Connect_PC_Client, self).__init__()
        self.name = name
        self.img_results = img_results
        self.img_valid = img_valid
        self.shoot_signal = shoot_signal
        # self.PC_close_signal = PC_close_signal
        self.setup_info = setup_info
        self.car_path = car_path
        self.server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # set the socket addr to be reusable
        self.host_name  = socket.gethostname()
        self.host_ip = socket.gethostbyname(self.host_name)
        print('RPi Wifi HOST IP:',self.host_ip)
        self.port = 10050
        self.socket_address = ('192.168.3.3',self.port)
        print('RPi Wifi Server Socket created')
        # bind the socket to the host. 
        #The values passed to bind() depend on the address family of the socket
        self.server_socket.bind(self.socket_address)
        print('Socket bind complete')
        #listen() enables a server to accept() connections
        #listen() has a backlog parameter. 
        #It specifies the number of unaccepted connections that the system will allow before refusing new connections.
        self.server_socket.listen(5)
        print('Wifi Socket now listening\n')
    
    def run(self):
        # assume the connection between PC and RPi is stable
        client_socket,addr = self.server_socket.accept()
        #client_socket.settimeout(5)
        print('Wifi Connection from:',addr)
        while True:
            try:
                '''
                 # Stage 1: Get obstacle info, calculate path command from android
                while True:
                    setup_from_tablet = self.setup_info.get()
                    print("working")
                    client_socket.send(setup_from_tablet)
                    s = setup_from_tablet.decode('UTF-8').strip()
                    if s == "path": # ** TBD
                        break
                        
                # Stage 2: Get complete path from PC
                data = b""
                payload_size = struct.calcsize(">L")
                while len(data) < payload_size:
                    packet = client_socket.recv(4096)
                    #if not packet: break
                    data+=packet
                packed_msg_size = data[:payload_size]
                data = data[payload_size:]
                msg_size = struct.unpack(">L",packed_msg_size)[0]
                while len(data) < msg_size:
                    data += client_socket.recv(4096)
                result_data = data[:msg_size]
                data = data[msg_size:]
                self.car_path.put(pickle.loads(result_data))
                print("Car path received from PC:", pickle.loads(result_data))
                '''
                
                # Stage 3: Wait for signal to take picture
                # ** The signal is the obstacle's id on the map
                while True:
                    if not self.shoot_signal.empty():
                        # get the obstacle id
                        img_map_id = self.shoot_signal.get()
                        # create the thread for streaming
                        stream_thread = Start_Stream("Stream Thread", client_socket)
                        # create the thread for receiving img recog results
                        img_result_thread = Receive_Img_Results("Img Result Thread", client_socket, img_map_id, \
                                                                self.img_results, self.img_valid)

                        # start threads
                        stream_thread.start()
                        print('Waiting for the stream thread to finish ...')
                        img_result_thread.start()
                        print('Waiting for the img result thread to finish ...')

                        stream_thread.join()
                        img_result_thread.join()
                        #res = self.img_results.get()
                        #print(res)
                    else:
                        time.sleep(0.1)
           
            except:
                client_socket.close()
                print('Wifi Socket now listening\n')
                client_socket,addr = self.server_socket.accept()
                client_socket.settimeout(5)
                print('Wifi Connection from:',addr)
       


class Start_Stream(threading.Thread):

    def __init__(self, name, client_socket):
        super(Start_Stream, self).__init__()
        self.name = name
        self.client_socket = client_socket

    def run(self):
        '''
        with picamera.PiCamera() as camera:
            camera.resolution = (320,240)
            camera.framerate = 24
            time.sleep(2)
            image = np.empty((240*320*3,),dtype=np.uint8)
            camera.capture(image,'bgr')
            image = image.reshape((240,320,3))
            '''
        
        vid = cv2.VideoCapture(0)
        vid.set(3,640) # CAP_PROP_FRAME_WIDTH 
        vid.set(4,480) # CAP_PROP_FRAME_HEIGHT 
        #vid.set(5,3) # CAP_PROP_FPS 
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY),90]
        #time.sleep(2)
        print("Ready to take pic")
        #while(vid.isOpened()):
        for i in range(1):
            res,frame = vid.read()
            frame = cv2.flip(frame,-1) # flip around both axes
            #cv2.imwrite('image'+str(0)+'.jpg', frame)
            result, frame = cv2.imencode('.jpg', frame, encode_param)
            a = pickle.dumps(frame,0) 
            message = struct.pack(">L",len(a))+a
            self.client_socket.sendall(message)
            print("Image sent")
            #cv2.imshow('Sending...',frame)
            #key = cv2.waitKey(10) 
            # print(key) # -1 will be returned if no key is pressed
            #if key == 27: # "ESC"
                #self.client_socket.close()
        vid.release()
        
class Receive_Img_Results(threading.Thread):

    def __init__(self, name, client_socket, img_map_id, results, img_valid):
        super(Receive_Img_Results, self).__init__()
        self.name = name
        self.client_socket = client_socket
        self.img_map_id = img_map_id
        self.results = results
        self.img_valid = img_valid
    
    def run(self):
        #'b' or 'B'produces an instance of the bytes type instead of the str type
        #used in handling binary data from network connections
        data = b""
        # Q: unsigned long long integer(8 bytes)
        payload_size = struct.calcsize(">L")
        #while True:
        while len(data) < payload_size:
            packet = self.client_socket.recv(4*1024)
            #if not packet: break
            data+=packet
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L",packed_msg_size)[0]
        while len(data) < msg_size:
            data += self.client_socket.recv(4*1024)
        result_data = data[:msg_size]
        data  = data[msg_size:]
        res = pickle.loads(result_data)
        if res == -1: # Nothing Detected!
            self.img_valid.put(False)
            print("Nothing Detected!")
        else:
            # get obstable id and class label
            result_str = str(self.img_map_id)+", "+res[1]
            self.results.put(result_str) 
            self.img_valid.put(True)
            print("Image recognition result:", result_str)
            # get bbox image
            #img_bbox = cv2.imdecode(res[0],cv2.IMREAD_COLOR)
            # ** add image idx on the map
            # cv2.imwrite('./image_result/image'+str(self.img_map_id)+'.jpg', img_bbox)
            
                
if __name__ == '__main__':
    img_results = Queue(maxsize=1)
    shoot_signal = Queue(maxsize=1)
    PC_close_signal = Queue(maxsize=1)
    PC_thread = Connect_PC_Client("PC Thread", img_results, shoot_signal, PC_close_signal)
    PC_thread.start()
    print("Waiting for PC thread to finish...")
    time.sleep(5)
    
    for i in range(3):
        t1 = time.time()
        # send the command to take picture
        # param: the img id on the map
        shoot_signal.put(i)
        # show current img recog results
        while img_results.empty():
            time.sleep(0.1)
        res = img_results.get()
        t2 = time.time()
        print("Image Rec Results:", res)
        print("Time taken:", t2-t1)
        print()
        time.sleep(3)
    PC_close_signal.put(1)
    PC_thread.join()

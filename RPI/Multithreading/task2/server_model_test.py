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

# Command
# cd /home/RP03/Desktop/Python\ Test
# python server_model_test.py 

#signal(SIGPIPE,SIG_DFL)
# Start server
class Connect_PC_Client(threading.Thread):

    def __init__(self, name, img_results):
        super(Connect_PC_Client, self).__init__()
        self.name = name
        self.img_results = img_results
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
        print('Socket now listening')

    def run(self):
        #t1 = 0
        while True:
            client_socket,addr = self.server_socket.accept()
            print('Connection from:',addr)
            
            #time.sleep(5)
            #client_socket.close()
            #print('Socket now listening')
            while True:
                input("Click for next image")
            
                # create the thread for streaming
                stream_thread = Start_Stream("Stream Thread", client_socket)
                # create the thread for receiving img recog results
                #img_result_thread = Receive_Img_Results("Img Result Thread", client_socket, self.img_results)

                # start threads
                stream_thread.start()
                print('Waiting for the stream thread to finish ...')
                #img_result_thread.start()
                #print('Waiting for the img result thread to finish ...')

                # keep updating img recog results
                '''
                while img_result_thread.is_alive():
                    self.img_results = img_result_thread.results
                    print(self.img_results)
                    time.sleep(0.1)'''

                stream_thread.join()
                #img_result_thread.join()
                #res = self.img_results.get()
                #print(res)
                #break



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
        vid.set(5,2) # CAP_PROP_FPS 
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY),90]
        time.sleep(2)
        print("Ready to take pic")
        while(vid.isOpened()):
        #for i in range(1):
            res,frame = vid.read()
            print("res:", res)
            frame = cv2.flip(frame,-1) # flip around both axes
            #cv2.imwrite('image'+str(i)+'.jpg', frame)
            result, frame = cv2.imencode('.jpg', frame, encode_param)
            a = pickle.dumps([0, frame],0) 
            message = struct.pack(">L",len(a))+a
            self.client_socket.sendall(message)
            #cv2.imshow('Sending...',frame)
            #key = cv2.waitKey(10) 
            # print(key) # -1 will be returned if no key is pressed
            #if key == 27: # "ESC"
            break
        self.client_socket.close()
        vid.release()
class Receive_Img_Results(threading.Thread):

    def __init__(self, name, client_socket, results):
        super(Receive_Img_Results, self).__init__()
        self.name = name
        self.client_socket = client_socket
        self.results = results
    
    def run(self):
        #'b' or 'B'produces an instance of the bytes type instead of the str type
        #used in handling binary data from network connections
        data = b""
        # Q: unsigned long long integer(8 bytes)
        payload_size = struct.calcsize(">L")
        while True:
            while len(data) < payload_size:
                packet = self.client_socket.recv(4096)
                #if not packet: break
                data+=packet
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack(">L",packed_msg_size)[0]
            while len(data) < msg_size:
                data += self.client_socket.recv(4096)
            result_data = data[:msg_size]
            data  = data[msg_size:]
            res = pickle.loads(result_data)
            if res == -1:
                self.results.put("Nothing Detected!")
            else:
                print(res)
                self.results.put(res)
                
if __name__ == '__main__':
    img_results = Queue(maxsize=1)
    PC_thread = Connect_PC_Client("PC Thread", img_results)
    PC_thread.start()
    print("Waiting for PC thread to finish...")
    # show current img recog results
    #while True:
        #if not img_results.empty():
        #print("Image Rec Results:", img_results.get())
    PC_thread.join()

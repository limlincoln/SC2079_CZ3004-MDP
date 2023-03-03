#import socket
import bluetooth
import threading
from queue import Queue
import cv2
import pickle
import struct
import time
import os
#import io
#import zlib
#from signal import signal, SIGPIPE, SIG_DFL
#import picamera
import numpy as np

from config import *


# Command
# cd /home/RP03/Desktop/Python\ Test/SC2079_CZ3004-MDP/RPI/Multithreading
# python A12_BLE.py 


class Connect_Android_Client(threading.Thread):

    def __init__(self, name, uuid, setup_info, run_cmd, IMG_label):
        super(Connect_Android_Client, self).__init__()
        self.name = name
        self.uuid = uuid
        self.setup_info = setup_info
        self.run_cmd = run_cmd
        self.IMG_label = IMG_label
        self.client_sock = None
 
        
        # make RPi discoverable
        os.system("sudo hciconfig hci0 piscan")
        os.system("sudo chmod o+rw /var/run/sdp")

    def send_msg(self, msg_str):
        self.client_sock.send(msg_str)
    
    def run(self):
        
        while True:
            self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.server_sock.bind(("", bluetooth.PORT_ANY))
            self.server_sock.listen(1)

            port = self.server_sock.getsockname()[1]
            
            bluetooth.advertise_service(self.server_sock, "SampleServer", service_id=self.uuid,
                                        service_classes=[self.uuid, bluetooth.SERIAL_PORT_CLASS],
                                        profiles=[bluetooth.SERIAL_PORT_PROFILE],
                                        # protocols=[bluetooth.OBEX_UUID]
                                        )

            print("Waiting for connection on RFCOMM channel", port)
            client_sock, client_info = self.server_sock.accept()
            self.client_sock = client_sock
            print("Accepted bluetooth connection from", client_info)
            
            try:
                while True:
                    data = client_sock.recv(1024)
                    s = data.decode('UTF-8').strip()
                    print("\nReceived via bluetooth: ", s)
                    print(("\nType a message to send to android:"))
                    if s == "end": 
                        break
        
            except OSError:
                pass
            
            print("Bluetooth disconnected.")

            client_sock.close()
            self.server_sock.close()
            print("Bluetooth socket closed.")
        
        

if __name__ == '__main__':
    
    
    # Create Queue objects for tranferring data among threads
    img_label = Queue(maxsize=1)
    img_valid = Queue(maxsize=1)
    shoot_signal = Queue(maxsize=1)
    # PC_close_signal = Queue(maxsize=1)
    STM_msg = Queue(maxsize=1)
    run_cmd = Queue(maxsize=1)
    setup_info = Queue(maxsize=1)
    car_path = Queue(maxsize=1)
    
    # Start threads
    BLE_thread = Connect_Android_Client("BLE Thread", UUID, setup_info, run_cmd, img_label)
    BLE_thread.start()
    
    while not BLE_thread.client_sock:
        time.sleep(0.1)
    
    while True:
        msg = input("Type a message to send to android:")
        BLE_thread.send_msg(msg)
    
    # Wait for threads to end
    BLE_thread.join()
    
    
    
    
    '''
    for i in range(3):
        t1 = time.time()
        # send the command to take picture
        shoot_signal.put(i) # param: the img id on the map
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
    '''


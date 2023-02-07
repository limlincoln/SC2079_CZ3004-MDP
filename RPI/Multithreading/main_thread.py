#import socket
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
from PC_thread import *
from bluetooth_thread import *


# Command
# cd /home/RP03/Desktop/Python\ Test/SC2079_CZ3004-MDP/RPI/Multithreading
# python main_thread.py 

# Git Command
# git pull 
# git add -A
# git commit -m ""
# git push

if __name__ == '__main__':
    if not os.path.exists("image_result"):
        os.makedirs("image_result")
    
    # Create Queue objects for tranferring data among threads
    img_results = Queue(maxsize=1)
    shoot_signal = Queue(maxsize=1)
    PC_close_signal = Queue(maxsize=1)
    
    # Start threads
    PC_thread = Connect_PC_Client("PC Thread", img_results, shoot_signal, PC_close_signal)
    PC_thread.start()
    BLE_thread = Connect_Android_Client("BLE Thread", UUID, BLE_PORT, shoot_signal)
    BLE_thread.start()
    time.sleep(5)
    
    # show current img recog results
    for i in range(3):
        while img_results.empty():
            time.sleep(0.1)
        res = img_results.get()
        print("Image Rec Results:", res)
    PC_close_signal.put(1)
    
    # Wait for threads to end
    PC_thread.join()
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


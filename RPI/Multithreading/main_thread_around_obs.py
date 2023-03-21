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
from PC_thread_around_obs import *
from bluetooth_thread import *

from STM_thread import *


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
    img_label = Queue(maxsize=1)
    img_valid = Queue(maxsize=1)
    shoot_signal = Queue(maxsize=1)
    # PC_close_signal = Queue(maxsize=1)
    STM_msg = Queue(maxsize=1)
    
    run_cmd = Queue(maxsize=1)
    setup_info = Queue(maxsize=1)
    car_path = Queue(maxsize=1)
    
    # Start threads
    STM_thread = Connect_STM_Client("STM Listen Thread", STM_msg)
    STM_thread.start()
    PC_thread = Connect_PC_Client("PC Thread", img_label, img_valid, shoot_signal, setup_info, car_path)
    #PC_thread = Connect_PC_Client("PC Thread", img_label, img_valid, shoot_signal)
    PC_thread.start()
    #BLE_thread = Connect_Android_Client("BLE Thread", UUID, setup_info, run_cmd, img_label)#, STM_thread, shoot_signal)
    #BLE_thread.start()
    time.sleep(5)
    
    stop_signal = True
    end_signal = False
    while not end_signal:
        if stop_signal:
            stop_signal = False
            # take a pic
            print("ready to take a picture")
            shoot_signal.put(0)
            is_img_valid = img_valid.get()
            if is_img_valid: # found the target image
                end_signal = True
            else: # send the move command to the next side
                STM_thread.send_msg("m")
                # wait for the STM signal indicating that the car has stopped
                while not stop_signal:
                    print("here")
                    s = STM_msg.get().strip()
                    print(len(s))
                    print("Msg from STM", s)
                    if s:
                        stop_signal = True
                        s = None
                
                
            
    
    '''
    # First, wait for the car path and the run command 
    while car_path.empty():
        time.sleep(0.1)
    car_path_received = car_path.get() # ** data structrue TBD
    
    while run_cmd.empty():
        time.sleep(0.1)
    run_cmd_received = run_cmd.get()
    
    # Start sending commands to STM
    while True:
        # Suppose the shoot signal is activated
        #stop = STM_msg.get()
        key = input("Press Enter to take a picture ...")
        shoot_signal.put(0) # put obstacle id here
    
        # If image recognition is successful, send more commands to STM
        is_img_valid = img_valid.get()
        while not is_img_valid: # If nothing detected, let the car move a bit and take another picture
            #time.sleep(0.5)
            #shoot_signal.put(0)
            #is_img_valid = img_valid.get()
            break
    '''
    
    '''
    # show current img recog results
    for i in range(3):
        while img_results.empty():
            time.sleep(0.1)
        res = img_results.get()
        print("Image Rec Results:", res)
    PC_close_signal.put(1)
    '''
    # Wait for threads to end
    PC_thread.join()
    #BLE_thread.join()
    STM_thread.join()
    
    
    
    
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


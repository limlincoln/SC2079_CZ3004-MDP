
#import socket
import threading
from queue import Queue
import cv2
import pickle
import struct
import time
import os
import math
#import io
#import zlib
#from signal import signal, SIGPIPE, SIG_DFL
#import picamera
import numpy as np

from config import *
from PC_thread2 import *
from bluetooth_thread2 import *

from STM_thread2 import *


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
    #setup_info = Queue(maxsize=1)
    #car_path = Queue(maxsize=1)
    
    # Start threads
    STM_thread2 = Connect_STM_Client("STM Listen Thread", STM_msg)
    STM_thread2.start()
    PC_thread = Connect_PC_Client("PC Thread", img_label, img_valid, shoot_signal, setup_info, car_path)
    PC_thread.start()
    #BLE_thread = Connect_Android_Client("BLE Thread", UUID, setup_info, run_cmd, img_label)#, STM_thread, shoot_signal)
    BLE_thread2 = Connect_Android_Client("BLE Thread", UUID, run_cmd)
    BLE_thread2.start()
    time.sleep(5)

    # Just take a picture
    # print("taking a pic")
    #shoot_signal.put(0)
    
    # First, wait for the car path and the run command 
    #print("Check1")
    #while car_path.empty():
    #time.sleep(0.1)
    #car_path_received = car_path.get() 
    #print("Check2")
    #while run_cmd.empty():
    #    time.sleep(0.1)
    #print("Check3")
    run_cmd_received = run_cmd.get()
    #print("Check4")
    # Get the complete path
    #complete_path = car_path_received[0]
    #complete_location = car_path_received[1]
    #cmd_count = 0
    # Start sending commands to STM
    #new_cmd_str = run_cmd_received + "," + "0"*num_padding
    STM_thread2.send_msg(run_cmd_received)
    print("Command sent to STM:",run_cmd_received)

    print("Waiting for stop signal")
    stop_msg = STM_msg.get()
    if stop_msg == "Stop":
        shoot_signal.put(0) # put obstacle id here
        # If image recognition is successful, send more commands to STM
        print("Main thread waiting for img result")
        is_img_valid = img_valid.get()
        print("Main thread got the img result")
        if not is_img_valid:
            print("Img detection failed")

    '''
    for cmd_tuple in complete_path:
        print("Entered")
        obs_id = cmd_tuple[0]
        cmd_str = cmd_tuple[1]
        cmd_list = cmd_str.split(",")
        #new_cmd_str = "".join(cmd_list)
        num_padding = 50 - len(cmd_str) - 1
        new_cmd_str = cmd_str + "," + "0"*num_padding
        #location_list = cmd_tuple[2]
        #cmd_str = "".join(cmd_list)
        STM_thread.send_msg(new_cmd_str)
        print("Command sent to STM:", new_cmd_str)
        # For each command
        count = 0
        while(count != len(cmd_list)):
             print("Waiting for stop signal")
             stop_msg = STM_msg.get()
             count += (len(stop_msg)//4)
             for i in range((len(stop_msg)//4)):
                 BLE_thread.send_msg(complete_location[cmd_count])
                 cmd_count += 1 
       # for i in range(len(cmd_list)):
        #    print("Waiting for stop signal")
         #   stop_msg = STM_msg.get()
          #  if stop_msg == "Stop":
                # send current location to Android
                #BLE_thread.send_msg(location_list[i])
           #     BLE_thread.send_msg("dummy location")
            #    pass
        
        #key = input("Press Enter to take a picture ...")
        shoot_signal.put(obs_id) # put obstacle id here
        # If image recognition is successful, send more commands to STM
        print("Main thread waiting for img result")
        is_img_valid = img_valid.get()
        print("Main thread got the img result")
        if not is_img_valid:
            print("Img detection failed")
        
        move_bw_count = 0
        while not is_img_valid: # If nothing detected, let the car move a bit and take another picture
            # send the command to move backward a bit
            if move_bw_count >=1: # cannot move more than twice
                break 
            print("Adjust camera position command sent", move_bw_count)
            STM_thread.send_msg("b"+","+"0"*48)
            move_bw_count += 1
            stop_msg = STM_msg.get()
            print("STM adjusted the camera")
            shoot_signal.put(obs_id)
            is_img_valid = img_valid.get()
        # move forward if moved backwards before
        for i in range(move_bw_count):
            STM_thread.send_msg("s"+","+"0"*48)
            stop_msg = STM_msg.get()
            print("STM changed the camera back")
            # print("nothing detected for Obstacle", obs_id)
            break
        
    # image recognition stops
    shoot_signal.put(-1)
    '''
    # show current img recog results
    '''
    for i in range(5):
        while img_results.empty():
            time.sleep(0.1)
        res = img_results.get()
        print("Image Rec Results:", res)
    PC_close_signal.put(1)
    '''
    
    # Wait for threads to end
    PC_thread2.join()
    BLE_thread2.join()
    STM_thread2.join()
    
    
    
    
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


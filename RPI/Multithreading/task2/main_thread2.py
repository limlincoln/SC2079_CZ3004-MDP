
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
    PC_thread.start()
    BLE_thread = Connect_Android_Client("BLE Thread", UUID, setup_info, run_cmd, img_label)#, STM_thread, shoot_signal)
    BLE_thread.start()
    time.sleep(5)
    
    direction = ""
    
    # First, wait for the run command 
    run_cmd_received = run_cmd.get()
   
    # Take the 1st pic
    shoot_signal.put(0)
    
    # Send first g cmd
    cmd = 'g' + ',' + "0"*48
    STM_thread.send_msg(cmd)
    
    # Wait for the 1st pic result
    is_img_valid = img_valid.get()
    img_id = img_label.get()
    print("img_id="+img_id)
    if img_id == 'TARGET, 0, 39':
        direction = "y"
    elif img_id == 'TARGET, 0, 38':
        direction = "z"
    
    # wait for run command
    #run_cmd_received = run_cmd.get()
    # send stm g signal 
    #cmd = 'g' + ',' + "0"*48
    #STM_thread.send_msg(cmd)
    #t1 = time.perf_counter_ns()
    stop_msg = STM_msg.get()
    #if stop_msg:
        #t2 = time.perf_counter_ns()
    #t = int(round(t2-t1, 3)*1000000)
    #t=int(((t2-t1)/1000000)-2500)
    #if t <= 0:
        #t = 0
    #print(" t1: "+str(t1))
    #print(" t2: "+str(t2)) 
    #print("Time taken: "+str(t))
    #direction = "y"
    if direction == "y" or direction == "z":
        print("No Need to Retake")
        cmd = str(direction) + "," + "0"*48
        STM_thread.send_msg(cmd)
    else:
        print("Take Picture again") 
        shoot_signal.put(0)
        is_img_valid = img_valid.get()
        img_id = img_label.get()
        print("img_id="+img_id)
        if img_id == 'TARGET, 0, 39':
            direction = "y"
        elif img_id == 'TARGET, 0, 38':
            direction = "z"
    cmd = str(direction) + "," + "0"*48
    STM_thread.send_msg(cmd)
    direction = ""
    stop_msg = STM_msg.get()
    if stop_msg:
         # get 2nd image
         shoot_signal.put(1)
         # send 2nd g cmd
         cmd = "g," + "0"*48
         STM_thread.send_msg(cmd)
         t1 = time.perf_counter_ns()
         stop_msg = STM_msg.get()
         t2 = time.perf_counter_ns()
         #t = int(((t2-t1)/1000000)-2500)
         
         # wait for 2ng img reault
         is_img_valid = img_valid.get()
         img_id = img_label.get()
         print("img_id="+img_id)
         if img_id == 'TARGET, 1, 39':
             direction = "y"
         elif img_id == 'TARGET, 1, 38':
             direction = "z"
             
    '''
    # send stm g signal 
    #cmd = 'g' + ',' + "0"*48
    STM_thread.send_msg(cmd)
    t1 = time.perf_counter_ns()
    stop_msg = STM_msg.get()
    if stop_msg:
        t2 = time.perf_counter_ns()
    #t = int(round(t2-t1, 3)*1000000)
    '''
    t=int(((t2-t1)/1000000)-2500) # change the time offset here
    if t <= 0:
        t = 0
    print(" t1: "+str(t1))
    print(" t2: "+str(t2)) 
    print("Time taken: "+str(t))
    #direction = "y"
    if direction == "y" or direction == "z":
        print("No Need to Retake")
        cmd = "t" +str(t) + "," + str(direction) + "," + "q"+"," +"x"+","+ "0"*(42-len(str(t)))
        STM_thread.send_msg(cmd)
    else:
        print("Take Picture again") 
        shoot_signal.put(1)
        is_img_valid = img_valid.get()
        img_id = img_label.get()
        print("img_id="+img_id)
        if img_id == 'TARGET, 1, 39':
            direction = "y"
        elif img_id == 'TARGET, 1, 38':
            direction = "z"
    cmd = "t" +str(t) + "," + str(direction) + "," + "q"+"," +"x"+","+ "0"*(42-len(str(t)))
    STM_thread.send_msg(cmd)

       		
    '''
    # Send start signal to STM
    cmd = direction + "," + "0"*48
    print(direction)
    STM_thread.send_msg(cmd)
    print(cmd)
    print("Start signal sent. Waiting for stop signal ...")
    '''
    '''
    # Record and send time 1st
    stop_msg = STM_msg.get()
    if stop_msg:
        t1 = time.time()
    stop_msg = STM_msg.get()
    if stop_msg:
        t2 = time.time()
    t = int(round(t2-t1, 3)*1000)
    cmd = "t," + str(t) + "," + "0"*(47-len(str(t)))
    STM_thread.send_msg(cmd)
    print("1st time (ms) sent:", t)
    '''
    '''
    # Record and send time 2nd
    stop_msg = STM_msg.get()
    if stop_msg:
        t1 = time.time()
    stop_msg = STM_msg.get()
    if stop_msg:
        t2 = time.time()
    t = int(round(t2-t1, 3)*1000)
    cmd = "t," + str(t) + "," + "0"*(47-len(str(t)))
    STM_thread.send_msg(cmd)
    print("2nd time (ms) sent:", t)
    
    # Take the 2nd pic
    shoot_signal.put(1)
    is_img_valid = img_valid.get()
    img_id = img_label.get()
    if img_id == 39:
        direction = "l"
    elif img_id == 38:
        direction = "r"
    cmd = direction + "," + "0"*48
    STM_thread.send_msg(cmd)
    '''
    '''
    
    # Start sending commands to STM
    for cmd_tuple in complete_path:
        obs_id = cmd_tuple[0]
        cmd_str = cmd_tuple[1]
        cmd_list = cmd_str.split(",")
        new_cmd_str = "".join(cmd_list)
        #location_list = cmd_tuple[2]
        
        STM_thread.send_msg(new_cmd_str)
        print("Command sent to STM:", new_cmd_str)
        
        for i in range(len(cmd_list)):
            print("Waiting for stop signal")
            stop_msg = STM_msg.get()
            if stop_msg == "Stop":
                # send current location to Android
                #BLE_thread.send_msg(location_list[i])
                #BLE_thread.send_msg("dummy location")
                pass
        
        # take a picture 
        shoot_signal.put(obs_id) # put obstacle id here
        # If image recognition is successful, send more commands to STM
        is_img_valid = img_valid.get()
        move_bw_count = 0
        while not is_img_valid: # If nothing detected, let the car move a bit and take another picture
            # send the command to move backward a bit
            if move_bw_count >=2: # cannot move more than twice
                break 
            print("Adjust camera position command sent", move_bw_count)
            STM_thread.send_msg("")
            move_bw_count += 1
            shoot_signal.put(obs_id)
            is_img_valid = img_valid.get()
        # move forward if moved backwards before
        for i in range(move_bw_count):
            STM_thread.send_msg("")
            
            # print("nothing detected for Obstacle", obs_id)
            break
    
    '''

    # Wait for threads to end
    PC_thread.join()
    BLE_thread.join()
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


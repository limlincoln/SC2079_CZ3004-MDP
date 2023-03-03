#!/usr/bin/env python3
"""PyBluez simple example rfcomm-server.py
Simple demonstration of a server application that uses RFCOMM sockets.
Author: Albert Huang <albert@csail.mit.edu>
$Id: rfcomm-server.py 518 2007-08-10 07:20:07Z albert $
"""

# Command
# cd /home/RP03/Desktop/Python\ Test/SC2079_CZ3004-MDP/RPI/Multithreading
# python bluetooth_thread.py 

# To get uuid
# sudo blkid

# Somehow fixed the bluetooth permission denied error
# sudo chmod o+rw /var/run/sdp

import os
import bluetooth
import threading
from queue import Queue
import time

from config import *



class Connect_Android_Client(threading.Thread):

    def __init__(self, name, uuid, setup_info, run_cmd, IMG_label):
        super(Connect_Android_Client, self).__init__()
        self.name = name
        self.uuid = uuid
        self.setup_info = setup_info
        self.run_cmd = run_cmd
        self.IMG_label = IMG_label
        #self.Andriod_msg = Andriod_msg
 
        
        # make RPi discoverable
        os.system("sudo hciconfig hci0 piscan")
        os.system("sudo chmod o+rw /var/run/sdp")

    def send_msg(self, msg_str):
        self.client_sock.send(msg_str)
    
    def run(self):
        is_cal_path_cmd_received = False
        is_run_cmd_received = False
        
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
                while not is_cal_path_cmd_received:
                    data = client_sock.recv(1024)
                    s = data.decode('UTF-8').strip()
                    print("Received via bluetooth: ", s)
                    self.setup_info.put(data)
                    if s == "path": # ** TBD
                        is_cal_path_cmd_received = True
                        
                while not is_run_cmd_received:
                    data = client_sock.recv(1024)
                    s = data.decode('UTF-8').strip()
                    print("Received via bluetooth:", s)
                    self.run_cmd.put(data)
                    if s == "run": # ** TBD
                        is_run_cmd_received = True
                '''
                while True:
                    if self.empty():
                        #time.sleep(0.1)
                    #stm_msg = self.STM_msg.get()
                    if not self.STM_msg.empty()
                        stm_msg = self.STM_msg.get()
                        client_sock.send(stm_msg)
                    #elif not self.shoot_signal.empty()
                    #   pc_msg = self.shoot_signal.get()
                    #   client_sock.send()
                     elif s.startswith("tp"):
                        img_map_id = s.split("tp")[-1]
                        self.shoot_signal.put(img_map_id)
                        print("Take Picture Command sent")
                    
                    data = client_sock.recv(1024)
                    #client_sock.send('Hello') 
                    if not data:
                        break
                    s = data.decode('UTF-8').strip()
                    print("Received", s)
                    self.setup_info.put(s)
                    #self.Andriod_msg.put(s)
                    # process the received string
                    if s == "end":
                        break
                '''
                while True:
                    if self.IMG_label.empty():
                        time.sleep(0.1)
                    else:
                        client_sock.send(self.IMG_label.get())
        
            except OSError:
                pass
            
            print("Bluetooth disconnected.")

            client_sock.close()
            self.server_sock.close()
            print("Bluetooth socket closed.")
        
        
if __name__ == '__main__':
    shoot_signal = Queue(maxsize=1)
    
    BLE_thread = Connect_Android_Client("BLE Thread", UUID, BLE_PORT, shoot_signal)
    BLE_thread.start()
    BLE_thread.join()
    print("Bluetooth thread ended")



    
    

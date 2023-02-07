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

from config import *



class Connect_Android_Client(threading.Thread):

    def __init__(self, name, uuid, port, shoot_signal):
        super(Connect_Android_Client, self).__init__()
        self.name = name
        self.uuid = uuid
        self.port = port
        self.shoot_signal = shoot_signal
        #self.results = results
        
        # make RPi discoverable
        os.system("sudo hciconfig hci0 piscan")

        self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.server_sock.bind(("", self.port))
        self.server_sock.listen(1)

        port = self.server_sock.getsockname()[1]
        
        bluetooth.advertise_service(self.server_sock, "SampleServer", service_id=uuid,
                                    service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                                    profiles=[bluetooth.SERIAL_PORT_PROFILE],
                                    # protocols=[bluetooth.OBEX_UUID]
                                    )

        print("Waiting for connection on RFCOMM channel", port)
    
    def run(self):
        client_sock, client_info = self.server_sock.accept()
        print("Accepted connection from", client_info)

        try:
            while True:
                data = client_sock.recv(1024)
                if not data:
                    break
                s = data.decode('UTF-8').strip()
                print("Received", s)
                # process the received string
                if s == "end":
                    break
                elif s.startswith("tp"):
                    img_map_id = s.split("tp")[-1]
                    self.shoot_signal.put(img_map_id)
                    print("Take Picture Command sent")
        except OSError:
            pass

        print("Disconnected.")

        client_sock.close()
        self.server_sock.close()
        print("All done.")
        
        
if __name__ == '__main__':
    shoot_signal = Queue(maxsize=1)
    
    BLE_thread = Connect_Android_Client("BLE Thread", UUID, BLE_PORT, shoot_signal)
    BLE_thread.start()
    BLE_thread.join()
    print("Bluetooth thread ended")



    
    

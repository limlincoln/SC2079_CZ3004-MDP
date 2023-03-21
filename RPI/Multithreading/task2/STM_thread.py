import serial
import threading
import time
from queue import Queue

from config import *

# Command
# cd /home/RP03/Desktop/Python\ Test/SC2079_CZ3004-MDP/RPI/Multithreading
# python STM_thread.py 


class Connect_STM_Client(threading.Thread):

    def __init__(self, name, STM_msg):
        super(Connect_STM_Client, self).__init__()
        self.name = name
        self.STM_msg = STM_msg
        #self.move_cmd = move_cmd
        self.ser = serial.Serial("/dev/ttyUSB0", parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, \
                            bytesize=serial.EIGHTBITS, baudrate=115200, timeout=1)
        
    def send_msg(self, message):
        self.ser.write(message.encode())
    
    def receive_message(self):
        message = self.ser.readline().strip().decode("utf-8")
        return message
        
    def run(self):
        
        while True:
            received_message = self.receive_message()
            if received_message != "":
                self.STM_msg.put(received_message) # ** could be shoot signal
                print("Received STM message:", received_message)

        self.ser.close()


if __name__ == '__main__':
    STM_msg = Queue(maxsize=1)
    
    STM_thread = Connect_STM_Client("STM Listen Thread", STM_msg)
    STM_thread.start()
    time.sleep(2)
    while True:
        msg = input("Type a message to send to STM:")
        STM_thread.send_msg(msg)
    
    STM_thread.join()
    print("STM Listen thread ended")

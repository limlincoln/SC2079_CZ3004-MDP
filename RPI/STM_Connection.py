import serial
import threading
from queue import Queue

from config import *


class Connect_STM_Client(threading.Thread):

    def __init__(self, name, STM_msg, move_cmd):
        super(Connect_Android_Client, self).__init__()
        self.name = name
        self.STM_msg = STM_msg
        self.move_cmd = move_cmd
        
    def send_message(message):
        ser.write(message.encode())

    def receive_message():
        print("testing")
        message = ser.readline().strip().decode("utf-8")
        #print(message)
        return message
    
    def run(self):
        ser = serial.Serial("/dev/ttyUSB0", parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS, baudrate=115200, timeout=1)
        
        send_message("Test")
        while True:
            received_message = receive_message()
            STM_msg.put(received_message)
            print("Received message:", received_message)

        ser.close()


if __name__ == '__main__':
    STM_msg = Queue(maxsize=1)
    STM_msg = Queue(maxsize=1)
    
    STM_thread = Connect_STM_Client("STM Listen Thread", STM_msg, STM_msg)
    STM_thread.start()
    STM_thread.join()
    print("STM Listen thread ended")

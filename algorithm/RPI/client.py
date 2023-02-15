# importing libraries
import socket
import cv2
import pickle
import struct
import os
# import imutils
# from signal import signal, SIGPIPE, SIG_DFL
from ultralytics import YOLO

# Config
class_label = ['1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'Bullseye', 'C', 'D', 'Down', 'E', 'F', 'G', 'H',
               'Left', 'Right', 'S', 'Stop', 'T', 'U', 'Up', 'V', 'W', 'X', 'Y', 'Z']
Bullseye_Index = class_label.index("Bullseye")
Image_Width_Center = 320 / 2

# signal(SIGPIPE,SIG_DFL)
# Client socket
# create an INET, STREAMing socket :
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# host_ip = '<localhost>'# Standard loopback interface address (localhost)
host_ip = '192.168.1.95'  # put the ip of the server here
port = 13050  # Port to listen on (non-privileged ports are > 1023)
# now connect to the web server on the specified port number
client_socket.connect((host_ip, port))
# 'b' or 'B'produces an instance of the bytes type instead of the str type
# used in handling binary data from network connections
data = b""
# Q: unsigned long long integer(8 bytes)
payload_size = struct.calcsize(">L")
# print("payload_size: {}".format(payload_size))
# Load Yolo v8 model
model = YOLO("./best_v8s_new.pt")

new_letter = "T"
# if not os.path.exists("./new_image/"+new_letter):
#     os.makedirs("./new_image/"+new_letter)
#     os.makedirs("./new_image/"+new_letter+"_result")

# Receive stream frames
i = 0
while True:
    # for i in range(60):
    # print('testing')
    while len(data) < payload_size:
        packet = client_socket.recv(4 * 1024)
        if not packet: break
        data += packet
    # print(len(data))
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]
    while len(data) < msg_size:
        data += client_socket.recv(4 * 1024)
    frame_data = data[:msg_size]
    data = data[msg_size:]
    frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
    frame = cv2.resize(frame, (640, 640))
    # cv2.imshow("Receiving...",frame)
    # cv2.imwrite("./new_image/"+new_letter+"/img_"+new_letter+"_"+str(i)+".jpg", frame)
    results = model.predict(show=True, source=frame, save=False, save_txt=False)
    # cv2.imwrite("./new_image/"+new_letter+"_result/img_"+new_letter+"_"+str(i)+".jpg", frame)
    i += 1
    print("--- Results ---")
    for box in results[0].boxes:
        print(class_label[int(box.cls.tolist()[0])], box.conf.tolist()[0])
    # key = cv2.waitKey(10) # -1 will be returned if no key is pressed
    # if key  == 27: # press "ESC" to end connection
    #     client_socket.close()
    #     break
client_socket.close()

# Model accuracy record:
"""
如果有 S 和其他字符被同时识别， 去掉S
如果 G 和 6 被同时识别，取 6
3: 需更多正面图片（识别成 S）
6: 需更多正面图片（识别成 G）
8: 需更多正面图片（识别成 S）
E: 需更多正面图片（无法识别）
D: 需要更多正面、右倾图片（无法识别）
T: 需更多正面图片（无法识别）
"""

#     if results:
#         #input(results)
#         result_list = []

#         for box in results[0].boxes:
#             box_info = [box.xywh, box.conf, box.cls]
#             result_list.append(box_info)
#         if len(result_list) == 0:
#             result_list = "Nothing detected!"

#         a = pickle.dumps(result_list)
#         message = struct.pack(">L",len(a))+a
#         # print(message)
#         client_socket.sendall(message)
#         print("Results sent:")
#         print(result_list)
#     key = cv2.waitKey(10) # -1 will be returned if no key is pressed
#     if key  == 27: # press "ESC" to end connection
#         break
# client_socket.close()
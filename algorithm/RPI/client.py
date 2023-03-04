import socket
import cv2
import pickle
import struct
import math
import time
import os
import glob

import numpy as np

import algorithm.RPI.clientConfig as config
from ultralytics import YOLO
#from algorithm.algo.TSP import NearestNeighbour
from algorithm.algo.TSPV2 import NearestNeighbour
from algorithm.algo.Environment import StaticEnvironment, AdvancedEnvironment
from algorithm.Entities.Obstacle import Obstacle
import re as re
import algorithm.settings as settings
from algorithm.constants import DIRECTION
from algorithm.algo.HStar import Node

class Client:
    """
    Client used to connect to RPI server and receive data for image recognition
    """

    def __init__(self, address, port):
        """
        Constructor for Client
        :param address: string
        :param port: int
        """
        self.model = YOLO("./best_v8s_new.pt")
        self.address = address
        self.port = port
        self.TSP = None
        self.env = None
        self.ObList = {}
        self.final_result = []
        self.class_label = config.CLASS_LABEL
        self.class_dcp = config.CLASS_DCP
        self.class_id = config.CLASS_ID
        self.bullseye_index = self.class_label.index("Bullseye")
        self.s_index = self.class_label.index("S")
        self.g_index = self.class_label.index("G")
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        path = "result_img"
        if not os.path.exists(path):
            os.makedirs(path)

    def process_single_img(self, result_list):
        """
        Main method for image Recognition
        :param result_list: image
        :return:
        """
        box_list = []
        box_position = []
        for box in result_list[0][0].boxes:
            if box.cls == self.bullseye_index:
                continue
            box_list.append(box)
            box_position.append(
                abs(config.IMAGE_WIDTH_CENTER - box.xywh.tolist()[0][0]))  # box.conf [box.xywh, box.cls]
        if not box_position:  # Nothing detected => need to move a bit to take another picture
            return -1
        if len(box_list) > 1:
            print("Multiple letters detected")
            # remove "S" and "G" if multiple letters are detected
            for i in reversed(range(len(box_list))):
                box = box_list[i]
                print(box.cls.tolist())
                cls_idx = int(box.cls.tolist()[0])
                if cls_idx == self.s_index or cls_idx == self.g_index:
                    box_list.pop(i)
                    box_position.pop(i)
        # ** TBD: see if distant cards will be captured
        box_idx = box_position.index(min(box_position))
        box = box_list[box_idx]

        bbox_xyxy = box.xyxy.tolist()[0]
        # bbox_xyxy[1] = bbox_xyxy[1] / Image_Height_Change_Ratio
        # bbox_xyxy[3] = bbox_xyxy[3] / Image_Height_Change_Ratio
        label_idx = int(box.cls.tolist()[0])
        return [bbox_xyxy, label_idx]

    def connect(self):
        """
        Attempt to connect to server
        :return:
        """
        try:
            print("Attempting to connect to " + self.address + "....")
            self.socket.connect((self.address, self.port))
            print("Connected")
        except:
            print("Error Connecting")

    def get_obstacles(self):
        """
        Listen to server for Obstacle input
        :return:
        """
        print("yoo")
        while True:
            print("testest")
            data = self.socket.recv(4 * 1024)
            if not data:
                continue
            s = data.decode('UTF-8').strip()
            print("Received from RPi:", s)
            if s == "run":
                break
            self.obstacle_string_converter(s)
        return

    def obstacle_string_converter(self, s: str):
        """
        converts string to useable tuple for Obstacle object instantiation
        :param s: string
        :return: tuple
        """
        s_to_list = s.split(',')
        values = re.findall(r'\d+', s)
        if len(s_to_list) == 3:
            if self.ObList.get(values[0]) is None:
                self.ObList[values[0]] = [(int(values[1])*10,int(values[2])*10), None, (settings.BLOCK_SIZE, settings.BLOCK_SIZE), values[0]]
            else:
                self.ObList[values[0]][0] = (int(values[1])*10, int(values[2])*10)
        else:
            direction  = re.findall(r'\b[A-Z]+(?:\s+[A-Z]+)*\b', s_to_list[1])
            if self.ObList.get(values[0]) is None:
                self.ObList[values[0]] = [None, direction[0], (settings.BLOCK_SIZE, settings.BLOCK_SIZE), values[0]]
            else:
                self.ObList[values[0]][1] = direction[0]


    def path_calculation(self):
        """
        Calculate Path using algorithm
        :return:
        """

        list_of_ob_objects :list(Obstacle) = []
        print(self.ObList)
        for key in self.ObList:
            values = self.ObList[key]
            print(values)
            list_of_ob_objects.append(Obstacle(values[0], values[1], values[2], values[3]))
        print(list_of_ob_objects)
        self.env = StaticEnvironment((200, 200), list_of_ob_objects)
        self.TSP = NearestNeighbour(self.env, (0, 0, DIRECTION.TOP, "P"))
        self.TSP.computeSequence()

        return self.TSP.getCommandList()

    def path_calculationV2(self):
        list_of_ob_objects: list(Obstacle) = []
        print(self.ObList)
        for key in self.ObList:
            values = self.ObList[key]
            print(values)
            list_of_ob_objects.append(Obstacle(values[0], values[1], values[2], values[3]))
        print(list_of_ob_objects)
        self.env = AdvancedEnvironment((200, 200), list_of_ob_objects)
        self.TSP = NearestNeighbour(self.env, (15, 15,DIRECTION.TOP.value))
        start = time.perf_counter()
        path = self.TSP.computeSequence()
        end = time.perf_counter()
        print("Time Taken:", end-start)
        coords = []
        for ob in path[0]:
            ob_coords = []
            for node in ob:
                ob_coords.extend(node.path)
            coords.append(ob_coords)
        self.TSP.dubins.save_path(coords)
        return path
    def send_path(self):
        """
        Send path to server
        :return:
        """
        path = self.path_calculationV2()
        string = self.convertToPath(path)
        a = pickle.dumps(string)
        message = struct.pack(">L", len(a)) + a
        self.socket.sendall(message)
        print("Car Path Sent")

    def receive_image(self):
        """
        Listen to server for image recognition
        :return: None
        """
        while True:
            # 'b' or 'B'produces an instance of the bytes type instead of the str type
            # used in handling binary data from network connections
            data = b""
            # Q: unsigned long long integer(8 bytes)
            payload_size = struct.calcsize(">L")

            # Receive stream frames
            result_list = []

            print('Waiting for img ...')
            while len(data) < payload_size:
                packet = self.socket.recv(4 * 1024)
                if not packet: break
                data += packet
            # t1 = time.time()
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack(">L", packed_msg_size)[0]
            while len(data) < msg_size:
                data += self.socket.recv(4 * 1024)
            frame_data = data[:msg_size]
            data = data[msg_size:]
            frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
            if frame == "all finished":
                break
            obs_id = frame[0]
            frame = frame[1]
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            resized_frame = cv2.resize(frame, (640, 640))
            # cv2.imwrite('image_T_'+str(21)+'.jpg', resized_frame)
            # t2 = time.time()
            # print("Time taken to receive the image:", t2-t1)
            results = self.model.predict(show=False, source=resized_frame, save=False, save_txt=False, device="cpu")
            result_list.append(results)

            # process single image
            res = self.process_single_img(result_list)
            if res == -1:
                final_result = res
                input("hold")
            else:
                tl = (int(res[0][0]), int(res[0][1] / config.IMAGE_HEIGHT_CHANGE_RATIO))
                br = (int(res[0][2]), int(res[0][3] / config.IMAGE_HEIGHT_CHANGE_RATIO))
                cv2.rectangle(frame, tl, br, (0, 255, 0), 2)
                text1 = self.class_dcp[res[1]]
                text2 = "Image ID: " + str(self.class_id[res[1]])
                tx = int(res[0][2])
                if tx + 175 > 639:
                    tx = int(res[0][0] - 175)
                    if tx < 0:
                        tx = 0
                ty = int((res[0][1] + res[0][3]) / 2 / config.IMAGE_HEIGHT_CHANGE_RATIO)
                # frame = cv2.resize(frame, (640,480))
                cv2.rectangle(frame, (tx + 5, ty - 25), (tx + 170, ty + 25), (255, 255, 255), -1)
                cv2.putText(frame, text1, (tx + 5, ty), 0, 0.8, (0, 255, 0), 1)
                cv2.putText(frame, text2, (tx + 5, ty + 20), 0, 0.8, (0, 255, 0), 1)
                cv2.imwrite('./result_img/image' + str(obs_id) + '.jpg', frame)
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                result, frame = cv2.imencode('.jpg', frame, encode_param)
                self.final_result = [self.class_id[res[1]], self.class_dcp[res[1]]]
            # send result
            a = pickle.dumps(self.final_result)
            message = struct.pack(">L", len(a)) + a
            self.socket.sendall(message)
            print("Results sent:")
            print(self.final_result)
            # client_socket.close()
            # break
            # key = cv2.waitKey(10) # -1 will be returned if no key is pressed
            # if key  == 27: # press "ESC" to end connection
            #     break

    def close(self):
        """
        Disconnect from socket
        :return:
        """
        self.socket.close()

    def display_images(self):
        """
        Display all the captured images
        :return:
        """
        col_list = []
        row_list = []
        # for i in range(len(self.ObList)):
        for img_path in glob.glob("./result_img/*.jpg"):
            image = cv2.imread(img_path)
            col_list.append(image)
            if len(col_list) % 3 == 0 and len(col_list) != 0:
                row = np.hstack(tuple(col_list))
                row_list.append(row)
                col_list = []

        stacked_image = np.stack(tuple(row_list))
        cv2.imshow("Stitched Image", stacked_image)
        cv2.waitKey('A')
        cv2.destoryAllWindows()


    def convertToPath(self, path):

        pathString = []
        for oneOb in path[0]:
            key = oneOb[-1].pos[3]
            tempString= ""
            coords = []
            for index , node in enumerate(oneOb):
                trail = ","
                if type(node.moves) == tuple:
                    for i, dubinsTuple in enumerate(node.moves[:3]):
                        if i == len(node.moves[:3])-1 and index == len(oneOb) - 1:
                            trail = ""
                        tempString += dubinsTuple + trail
                else:
                    if index == len(oneOb) - 1:
                        trail = ""
                    tempString += str(node.moves) + trail
                coords.append(node.path)
            pathString.append((key, tempString, coords))
        return pathString

    def run(self):
        """
        Start the server
        :return:
        """
        self.connect()
        self.get_obstacles()
        #self.path_calculationV2()
        self.send_path()
        self.receive_image()
        self.display_images()
        self.close()
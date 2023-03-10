import socket
import cv2
import pickle
import struct
import math
import time
import os
import glob

import numpy as np
import pygame.image

import algorithm.RPI.clientConfig as config
from ultralytics import YOLO
from algorithm.algo.TSP import NearestNeighbour1
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
        box_width = []
        for box in result_list[0][0].boxes:
            if box.cls == self.bullseye_index:
                continue
            box_list.append(box)
            box_position.append(
                abs(config.IMAGE_WIDTH_CENTER - box.xywh.tolist()[0][0]))  # box.conf [box.xywh, box.cls]
            box_width.append(box.xywh.tolist()[0][2])
        if not box_position:  # Nothing detected => need to move a bit to take another picture
            return -1
        '''
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
        '''
        # ** TBD: see if distant cards will be captured
        # box_idx = box_position.index(min(box_position))
        # box = box_list[box_idx]
        # select the image with the largest width
        box_idx = box_width.index(max(box_width))
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
        self.TSP = NearestNeighbour1(self.env, (0, 0, DIRECTION.TOP, "P"))
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
        string = self.path_calculation()
        #string = self.convertToPath(path)
        a = pickle.dumps(string)
        message = struct.pack(">L", len(a)) + a
        self.socket.sendall(message)
        print("Car Path Sent")

    def receive_image(self):
        """
        Listen to server for image recognition
        :return: None
        """
        # 'b' or 'B'produces an instance of the bytes type instead of the str type
        # used in handling binary data from network connections
        data = b""
        # Q: unsigned long long integer(8 bytes)
        payload_size = struct.calcsize(">L")

        while True:

            # Receive stream frames
            result_list = []

            print('Waiting for img ...')
            try:
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
                if len(frame) == 1:
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
                    self.final_result = res
                    print("Nothing detected")
                    # input("hold")
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
                    cv2.putText(frame, text1, (tx + 5, ty), 0, 0.8, (0, 0, 0), 2)
                    cv2.putText(frame, text2, (tx + 5, ty + 20), 0, 0.8, (0, 0, 0), 2)
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
            except Exception as e:
                print(e)
                print("yo we fked")
                return
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
        # set the path to the directory containing the images
        path = "./result_img"

        # set the desired dimensions for each image in the grid layout
        desired_width = 350
        desired_height = 350

        # create an empty list to store the images
        images = []

        # iterate over the files in the directory
        for file in os.listdir(path):
            # check that the file is an image file
            if file.endswith('.jpg') or file.endswith('.png'):
                # load the image using cv.imread()
                img = cv2.imread(os.path.join(path, file))
                # resize the image to the desired dimensions
                img = cv2.resize(img, (desired_width, desired_height), interpolation=cv2.INTER_CUBIC)
                # append the image to the list of images
                images.append(img)

        # calculate the number of rows and columns required for the grid layout
        num_images = len(images)
        num_rows = int(math.sqrt(num_images))
        num_cols = math.ceil(num_images / num_rows)

        # create an empty numpy array to hold the grid layout of the images
        grid = np.zeros((num_rows * desired_height, num_cols * desired_width, 3), dtype=np.uint8)

        # iterate over the list of images and insert each image into the grid layout array
        for i, img in enumerate(images):
            row = i // num_cols
            col = i % num_cols
            y = row * desired_height
            x = col * desired_width
            grid[y:y + desired_height, x:x + desired_width, :] = img

        # display the grid layout of images using cv.imshow()
        cv2.imshow("All Images", grid)

        # wait for a key press and then close the window
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def display_imageV2(self):
        images = []
        image_counter = 0
        for image in os.listdir("./result_img"):
            path = "result_img/" + image
            image_counter += 1
            new_image = pygame.image.load(path)
            trans_image = pygame.transform.smoothscale(new_image, (400, 400))
            images.append(trans_image)

        background_colour = (234, 212, 252)

        # Define the dimensions of
        # screen object(width,height)
        screen = pygame.display.set_mode((1600, 800))

        # Set the caption of the screen
        pygame.display.set_caption('images')

        # Fill the background colour to the screen
        screen.fill(background_colour)


        # Variable to keep our game loop running
        running = True
        grid_space = [(0,0), (400,0), (800,0), (1200,0), (1600,0), (0,400), (400,400), (800,400)]
        # game loop
        while running:

            for i in range(image_counter):
                x, y = grid_space[i]
                image = images[i]
                screen.blit(image, (x, y))
            pygame.display.flip()
            # for loop through the event queue
            for event in pygame.event.get():

                # Check for QUIT event
                if event.type == pygame.QUIT:
                    running = False




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
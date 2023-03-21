import cv2
import time

vs = cv2.VideoCapture(0)
time.sleep(2)
if vs.isOpened(): 
    res, frame = vs.read()
    print(res)

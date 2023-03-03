# Image Recognition

Class_label = ['1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'Bullseye', 'C', 'D', 'Down', 'E', 'F', 'G', 'H', 'Left', 'Right', 'S', 'Stop', 'T', 'U', 'Up', 'V', 'W', 'X', 'Y', 'Z']
Class_dcp = ["one", "two", "three", "four", "five", "six", "seven", "eight", "nine", \
            "Alphabet A", "Alphabet B", "Bullseye", "Alphabet C", "Alphabet D", \
            "down arrow", "Alphabet E", "Alphabet F", "Alphabet G", "Alphabet H", \
            "left arrow", "right arrow", "Alphabet S", "Stop", "Alphabet T", \
            "Alphabet U", "up arrow", "Alphabet V", "Alphabet W", "Alphabet X", \
            "Alphabet Y", "Alphabet Z"]
Class_id = [11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, -1, 22, 23, 37, 24, 25, 26, 27, 39, 38, 28, 40, 29, \
            30, 36, 31, 32, 33, 34, 35] # -1 for Bullseye
Bullseye_Index = Class_label.index("Bullseye")
S_Index = Class_label.index("S")
G_Index = Class_label.index("G")
Image_Width_Center = 640 / 2
Image_Height_Change_Ratio = 320 / 240


# Android & Bluetooth

UUID = "996c1b5f-170b-4f38-a5e0-85eef5acf12c"
BLE_PORT = 1

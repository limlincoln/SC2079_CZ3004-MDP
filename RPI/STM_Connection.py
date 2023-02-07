import serial

ser = serial.Serial("/dev/ttyACM0", baudrate=115200)

def send_message(message):
    ser.write(message.encode())

def receive_message():
    message = ser.readline().decode().strip()
    return message

send_message("Hello STM32F407")

received_message = receive_message()
print("Received message:", received_message)

ser.close()

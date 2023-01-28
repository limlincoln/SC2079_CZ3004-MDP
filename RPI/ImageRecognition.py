import socket
import picamera

# IP address and port of the computer to send the photo to
ip = '192.168.1.100'
port = 8000

# Create the socket and connect to the computer
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((ip, port))

# Take a photo with the Raspberry Pi camera
with picamera.PiCamera() as camera:
    camera.start_preview()
    camera.capture('photo.jpg')
    camera.stop_preview()

# Open the photo file in binary
with open('photo.jpg', 'rb') as f:
    # Send the photo file through the socket
    s.sendall(f.read())

# Close the socket
s.close()
print("Photo sent!")

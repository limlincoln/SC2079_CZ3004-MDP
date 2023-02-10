import bluetooth

def send_message(message, address):
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((address, 1))
    sock.send(message)
    sock.close()

def receive_message(address):
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((address, 1))
    message = sock.recv(1024).decode().strip()
    sock.close()
    return message

# Replace with the Bluetooth address of your Android tablet
#address = "90:EE:C7:E7:D3:C2"
#address = "AC:88:FD:37:3B:76"
address = "24:31:54:CC:1B:8B"

send_message("Hello Android tablet", address)

received_mes = receive_message(address)
print("Received message:", received_message)

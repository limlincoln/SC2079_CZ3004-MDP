import bluetooth
import os 
def send_message(message, address):
    '''
    service_matches = bluetooth.find_service(uuid=None, address=address)
    first_match = service_matches[0]
    input(first_match)
    if len(service_matches) == 0:
        print("Couldn't find the SampleServer service.")
        sys.exit(0)
    port = first_match["port"]
    name = first_match["name"]
    host = first_match["host"]
    print("Connecting to \"{}\" on {} with port {}".format(name, host, port))
    '''
    '''
    for i in range(25):
        try:
            port = i+1
            sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            sock.connect((address, port))
            sock.send(message)
            print("message sent")
        except:
            print("port " + str(port) + " refused")
            continue
    '''
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((address, 19))
    sock.send(message)
    
def receive_message(address):
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((address, 3))
    message = sock.recv(1024).decode().strip()
    # sock.close()
    return message

# Replace with the Bluetooth address of your Android tablet
#address = "90:EE:C7:E7:D3:C2"
#address = "AC:88:FD:37:3B:76"
address = "24:31:54:CC:1B:8B"
#address = "D0:04:B0:66:F6:42"

send_message("Hello Android tablet", address)

#received_mes = receive_message(address)
#print("Received message:", received_message)

print("Bluetooth disconnected.")

#sock.close()

send_message("Hello Android tablet again", address)

        
    

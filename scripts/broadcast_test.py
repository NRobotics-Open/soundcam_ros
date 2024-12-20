#!/usr/bin/env python3

import socket
from soundcam_protocol import CommandCodes, DataMessages, MDDataMessage, Device, \
    DataObjects, \
    CameraProtocol, Features, Status

protocol = CameraProtocol(protocol=3, debug=True)

# Configuration
BROADCAST_IP = "192.168.3.255"
UDP_SEND_PORT = 51914
UDP_RECV_PORT = 51915
MESSAGE = b"Hello AKAMs send your ID"
MESSAGE2 = b"SoundCams send your ID"

# Function to send a broadcast message
def send_broadcast():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as udp_socket:
        udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcasting
        udp_socket.sendto(MESSAGE2, (BROADCAST_IP, UDP_SEND_PORT))
        print(f"Broadcast message sent to {BROADCAST_IP}:{UDP_SEND_PORT}")

# Function to receive responses
def receive_responses():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_recv_socket:
        udp_recv_socket.bind(("", UDP_RECV_PORT))
        udp_recv_socket.settimeout(0.5)  # Set timeout for receiving responses
        print(f"Listening for responses on {BROADCAST_IP}:{UDP_RECV_PORT}...")

        try:
            while True:
                data, addr = udp_recv_socket.recvfrom(1024)  # Buffer size 1024 bytes
                print(f"Received message from {addr}: {data}")
                protocol.unpackDecodeResponse(data)
                print(f'Processed data: \n {protocol.getDeviceStatus()}')
        except socket.timeout:
            print("No more responses received. Exiting.")

if __name__ == "__main__":
    send_broadcast()
    receive_responses()

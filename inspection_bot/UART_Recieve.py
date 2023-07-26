import socket

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind(('192.168.0.87', 9002))

while True:
    data, address = udp_socket.recvfrom(12)
    print("Received data:", data.decode('utf-8'))


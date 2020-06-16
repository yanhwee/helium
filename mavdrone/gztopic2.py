import socket

from proto.diagnostics_pb2 import Diagnostics
from proto.

GAZEBO_IP = '192.168.1.246'
GAZEBO_PORT = 40117

BUFFER_SIZE = 40960

# Subscribe
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Hello')
s.connect((GAZEBO_IP, GAZEBO_PORT))
print('World')
while True:
    data = s.recv(BUFFER_SIZE)
    print(data)
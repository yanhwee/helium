import socket

GAZEBO_IP = '127.0.0.1'
GAZEBO_PORT = 11345

NODE_IP = '127.0.0.1'
NODE_PORT = 11451

s_reg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_reg.connect((GAZEBO_IP, GAZEBO_PORT))
s_reg.close()
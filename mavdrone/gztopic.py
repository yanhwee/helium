import socket
import time
from datetime import datetime

from proto.packet_pb2       import Packet
from proto.subscribe_pb2    import Subscribe
from proto.publish_pb2      import Publish
from proto.diagnostics_pb2  import Diagnostics

GAZEBO_IP = '127.0.0.1'
GAZEBO_PORT = 11345

NODE_IP = '127.0.0.1'
NODE_PORT = 11451

BUFFER_SIZE = 40960

# Listen

s_pub = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_pub.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s_pub.bind(('', 0))
s_pub.listen(5)

# Register

pk_sub = Subscribe()
pk_sub.topic = '/gazebo/default/diagnostics'
pk_sub.msg_type = Diagnostics.DESCRIPTOR.full_name
pk_sub.host = NODE_IP
pk_sub.port = NODE_PORT

pk = Packet()
pk.stamp.sec = datetime.now().second
pk.stamp.nsec = datetime.now().microsecond
pk.type = 'subscribe'
pk.serialized_data = pk_sub.SerializeToString()

s_reg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_reg.connect((GAZEBO_IP, GAZEBO_PORT))
s_reg.send(hex(pk.ByteSize()).rjust(8).encode('utf-8'))
s_reg.send(pk.SerializeToString())

# while True:
#     data = s_reg.recv(BUFFER_SIZE)
#     print(data)

# s_pub.setblocking(False)
while True:
    print('hi')
    conn, addr = s_pub.accept()
    print('addr:', addr)
    

try:
    print('hi')
    conn, addr = s_pub.accept()
    print('addr:', addr)

    # while True:
    data = conn.recv(BUFFER_SIZE)

    pk = Packet()
    pk.ParseFromString(data[8:])
    print('pk:', pk)

    pk_diag = Diagnostics()
    pk_diag.ParseFromString(pk.serialized_data)
    print('pk_diag:', pk_diag)

finally:
    s_reg.close()
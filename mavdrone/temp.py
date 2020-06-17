# pylint: skip-file
#!/usr/bin/env python

import socket
import time
import sys
from datetime import datetime

sys.path.append("./proto") # NEW
from proto.packet_pb2 import Packet
from proto.publish_pb2 import Publish
from proto.request_pb2 import Request
from proto.response_pb2 import Response
from proto.world_control_pb2 import WorldControl
from proto.subscribe_pb2 import Subscribe

MASTER_TCP_IP   = '127.0.0.1'
MASTER_TCP_PORT = 11345

NODE_TCP_IP     = '127.0.0.1'
NODE_TCP_PORT   = 11451

TCP_BUFFER_SIZE = 40960

# Listen for Subscribers
s_sub = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_sub.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s_sub.bind((NODE_TCP_IP, NODE_TCP_PORT))
s_sub.listen(5)

# Register as a Publisher with Gazebo
pk            = Packet()
pk.stamp.sec  = int(time.time())
pk.stamp.nsec = datetime.now().microsecond
pk.type       = "advertise"

pub           = Publish()
pub.topic     = "/gazebo/default/world_control" # OK
pub.msg_type  = WorldControl.DESCRIPTOR.full_name # OK
pub.host      = NODE_TCP_IP
pub.port      = NODE_TCP_PORT

pk.serialized_data = pub.SerializeToString()

s_reg = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s_reg.connect((MASTER_TCP_IP, MASTER_TCP_PORT))
s_reg.send(hex(pk.ByteSize()).rjust(8).encode('utf-8')) # Appended
s_reg.send(pk.SerializeToString())

conn, address = s_sub.accept()

# Pack Data for Reply
msg_world_control = WorldControl()
msg_world_control.multi_step = 143
msg_world_control.pause = True

# Publish Packet to Subscriber
pk_pub            = Packet()
pk_pub.stamp.sec  = int(time.time())
pk_pub.stamp.nsec = datetime.now().microsecond
pk_pub.type       = WorldControl.DESCRIPTOR.full_name # OK
pk_pub.serialized_data = msg_world_control.SerializeToString() # OK

def stepSimulation(nbOfSteps):
    # msg_world_control.multi_step = nbOfSteps # Ignore
    conn.send(hex(msg_world_control.ByteSize()).rjust(8).encode('utf-8')) # Appeneded
    conn.send(msg_world_control.SerializeToString()) # OK
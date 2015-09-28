#!/usr/bin/env python

import socket


host="192.168.42.185"
port=5555

  
sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((host,port))
while 1:
    data,addr = sock.recvfrom(1024)
    print(data)

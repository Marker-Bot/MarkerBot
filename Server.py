#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket

sock = socket.socket()
sock.bind(('', 9000))
sock.listen(1)
conn, addr = sock.accept()

print('connected:', addr)

while True:
    data = conn.recv(1024)
    data = data.decode()
    arr = data.split("|")
    print(arr)
    x = float(arr[0])
    y = float(arr[1])
    angle = float(arr[2])
    print(str(x) + "\n" + str(y) + "\n" + str(angle) + "\n")
    if not data:
        break
    conn.send("Success".encode())

conn.close()
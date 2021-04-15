#!/usr/bin/env python
# -*- coding: utf-8 -*-

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
x = input()
y = input()
angle = input()
arr = [x, y, angle]
myArr = ""
for i in arr:
    myArr = myArr + str(i) + "|"
myArr = myArr[: -1]
sock = socket.socket()
sock.connect(('ev3dev.local', 9000))
sock.send(myArr.encode())

data = sock.recv(1024).decode()
sock.close()

print(data)
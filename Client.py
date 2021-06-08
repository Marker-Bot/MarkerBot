#!/usr/bin/env python
# -*- coding: utf-8 -*-

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket

def sendData(x,y,flags):
    """
    Функция для отправки всех имеющихся данных (начальных позиций, координат x и y рисунка) на Brick EV3
    """
    startPositionXY = [0,0]


    sock = socket.socket()
    sock.connect(('ev3dev.local', 9000))
    sock.send(startPositionXY.encode())
    sock.send(x.encode())
    sock.send(y.encode())
    sock.send(flags.encode())

    data = sock.recv(1024).decode()
    sock.close()

    print(data)

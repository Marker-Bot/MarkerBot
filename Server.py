#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket

def receiveData():
    """
    Функция, которая принимает данные о координатах с клиента
    """
    sock = socket.socket()
    sock.bind(('', 9000))
    sock.listen(1)
    conn, addr = sock.accept()

    print('connected:', addr)

    while True:
        startPositionXY = conn.recv(1024).decode()
        x = conn.recv(1024).decode()
        y = conn.recv(1024).decode()
        flags = conn.recv(1024).decode()

        if not startPositionXY:
            break
        conn.send("Success".encode())

    conn.close()
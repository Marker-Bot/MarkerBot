#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket


def createSocket():
    """
    Функция, создающая socket для подключения
    :return: conn - подключение для принятия данных
    """
    sock = socket.socket()
    sock.bind(('', 9000))
    sock.listen(1)
    conn, addr = sock.accept()
    return conn


def receiveData(conn):
    """
    Функция, которая принимает данные о координатах с клиента
    :param conn: подключение для принятия данных
    :return: startPositionXY - arraylist начальных координат
             x - arraylist координат по X
             y - arraylist координат по Y
             flags - arraylist поднятия и опускания маркера
    """
    startPositionXY = conn.recv(1024).decode()
    x = conn.recv(1024).decode()
    y = conn.recv(1024).decode()
    flags = conn.recv(1024).decode()
    conn.send("Success".encode())

    return startPositionXY, x, y, flags


def closeConnection(conn):
    """
    Функция для закрытия соединения
    :param conn: подключение для принятия данных
    :return:
    """
    conn.close()

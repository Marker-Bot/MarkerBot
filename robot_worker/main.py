#!/usr/bin/env python3
from ev3dev.ev3 import *
import time
import math

from .Server import createSocket, receiveData, closeConnection


def filter(sp):
    """
    Функция для фильтрации значений напряжения

    :param sp: напряжение подаваемое на мотор
    :return: отфильтрованное напряжение
    """
    if abs(sp) > 100:
        sp = 100 * abs(sp) / sp
    return sp


def getmL():
    """
    Функция возвращающая позицию левого мотора

    :return:позиция левого мотора в радианах
    """
    return mL.position


def getmR():
    """
    Функция возвращающая позицию правого мотора

    :return:позиция правого мотора в радианах
    """
    return mR.position


def move_marker(getM, flag_down, iterations, mM):
    """
    Функция осуществляющая поднятие и опускание маркера в зависимости от рисунка

    :param getM: входной массив с метками рисовать или нет
    :param flag_down: флаг состояния маркера: опущен или поднят
    :param iterations: итерации цикла
    :param mM: мотор прикрепленный к маркеру
    """
    if not getM[i]:
        flag_down = flag_down * (-1)
    if iterations < 1000:
        mM.run_direct(duty_cycle_sp=int(flag_down * 100))
    else:
        mM.stop(stop_action="brake")


def main(Rad, Base, mL, mR, mM, fh, btn):
    """
    Основная функция, осуществляющая движения робота по заданному рисунку.

    :param getX: координаты X по которым движется робот
    :param getY: координаты Y по которым движется робот
    :param getM: флаги поднятия и опускания маркера
    :param Rad: радиус колес
    :param Base: расстояние между колесами
    :param mL: левый мотор
    :param mR: правый мотор
    :param mM: мотор маркера
    :param fh: файл записи логов
    :param btn: кнопка включения робота
    """
    conn = createSocket()

    currX = currY = currA = 0
    prefX = prefY = prefA = 0
    currmL = currmR = prefmL = prefmR = 0
    mL.position = mR.position = summ = 0
    mL.stop(stop_action='coast')
    mR.stop(stop_action='coast')
    flag_down = -1
    for k in range(1):
        start, getX, getY, getM = receiveData(conn)
        for i in range(len(getX)):
            iterations = 0
            try:
                while True:
                    iterations += 1
                    distance = math.pow(currX - getX[i], 2) + math.pow(currY - getY[i], 2)
                    kekX = (getX[i] - currX) * math.cos(currA) + (getY[i] - currY) * math.sin(currA)
                    kekY = (currX - getX[i]) * math.sin(currA) + (getY[i] - currY) * math.cos(currA)
                    if abs(kekX) < 0.1:
                        deltaA = math.pi / 2 * abs(kekY) / kekY
                    else:
                        deltaA = math.atan(kekY / kekX)
                    if kekX < 0:
                        if deltaA < 0:
                            deltaA = deltaA + math.pi
                        else:
                            deltaA = deltaA - math.pi
                    if distance < 10 or btn.any():
                        move_marker(getM, flag_down, iterations, mM)
                        break
                    else:
                        currmL = getmL()
                        currmR = getmR()
                        dmL = currmL - prefmL
                        dmR = currmR - prefmR
                        summ = math.radians((dmR + dmL) / 2) * Rad
                        currA = prefA + math.radians((dmR - dmL) * Rad / Base)
                        currX = prefX + math.cos(currA) * summ
                        currY = prefY + math.sin(currA) * summ
                        speed = 40 * (math.pow(math.e, -10 * deltaA * deltaA))
                        mL.run_direct(duty_cycle_sp=filter(speed - deltaA * 100))
                        mR.run_direct(duty_cycle_sp=filter(speed + deltaA * 100))
                        prefmL = currmL
                        prefmR = currmR
                        prefA = currA
                        prefX = currX
                        prefY = currY
                        fh.write(str(currX) + ' ' + str(currY) + '\n')
            finally:
                mL.stop(stop_action='brake')
                mR.stop(stop_action='brake')
                time.sleep(0.1)
            closeConnection(conn)
            fh.close


if __name__ == '__main__':
    Rad = 21.6
    Base = 185
    mL = LargeMotor('outB')
    mR = LargeMotor('outC')
    mM = MediumMotor('outA')
    fh = open('data.txt', 'w')
    fh.write("0 0" + '\n')
    btn = Button()
    main(getX, getY, getM, Rad, Base, mL, mR, mM, fh, btn)

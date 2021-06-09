#!/usr/bin/env python3
from ev3dev.ev3 import *
import time
import math

# getX = [100,100,200,300,300,500,500,520,540,560,580,600,500,540,580,600,640,680,660,640,620,600,500,0]
# getY = [100,400,300,400,100,100,400,390,380,350,330,300,300,280,260,240,220,200,180,160,140,120,100,0]
# getM=[True,True,True,True,False,False,True,True,True,True,True,True,True,True,True,True,True,True,True,True,True,True,False,True]
getX = [0, 200, 400, 0, 0, 400, 200, 0, 0]
getY = [100, 400, 100, 100, 300, 300, 0, 300, 0]
getM = [True, True, True, False, False, True, True, False, True]
Rad = 21.6
Base = 185
mL = LargeMotor('outB')
mR = LargeMotor('outC')
mM = MediumMotor('outA')
fh = open('data.txt', 'w')
fh.write("0 0" + '\n')
btn = Button()


def filter(sp):
    if abs(sp) > 100:
        sp = 100 * abs(sp) / sp
    return (sp)


def getmL():
    return (mL.position)


def getmR():
    return (mR.position)


currX = currY = currA = 0
prefX = prefY = prefA = 0
currmL = currmR = prefmL = prefmR = 0
mL.position = mR.position = summ = 0
mL.stop(stop_action='coast')
mR.stop(stop_action='coast')

iterations = 0
flag_down = -1
for k in range(1):
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
                    if not getM[i]:
                        # time.sleep(2)
                        # mM.stop(stop_action = 'brake')
                        flag_down = flag_down * (-1)
                    if iterations < 1000:
                        mM.run_direct(duty_cycle_sp=int(flag_down * 100))
                    else:
                        mM.stop(stop_action="brake")
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
        fh.close

#!/usr/bin/python3

from ev3dev.ev3 import *
import time
import math

getX = [1000]
getY = [0]
getM = [True]
Rad = 31.2
Base = 175.55

mM = LargeMotor('outC')
mL = LargeMotor('outB')
mR = LargeMotor('outA')
# sL = LightSensor('in2')
# sR = LightSensor('in1')
fh = open('data.txt', 'w')
fh.write("0 0" + '\n')
btn = Button()


def getmL():
    return (mL.position)


def getmR():
    return (mR.position)


def filter(sp):
    if abs(sp) > 100:
        sp = 100 * abs(sp) / sp
    return (sp)


# def minval():
#	return(min(sL.value(), sR.value()))
'''
def error():
	if minval() < 400:
		return((minval() - 400)*math.copysign(1, sR.value() - sL.value()))
	else:
		return(0)
'''
currX = currY = currA = 0
prefX = prefY = prefA = 0
currmL = currmR = prefmL = prefmR = 0
mL.position = mR.position = summ = 0

Vmax = 100
Kw = 60
Kp = 0.05
mL.stop(stop_action='coast')
mR.stop(stop_action='coast')

timest = time.time()

flag_down = False

for k in range(1):
    for i in range(len(getX)):
        try:
            while True:
                distance = math.sqrt(math.pow(currX - getX[i], 2) + math.pow(currY - getY[i], 2))
                kekX = (getX[i] - currX) * math.cos(currA) + (getY[i] - currY) * math.sin(currA)
                kekY = (currX - getX[i]) * math.sin(currA) + (getY[i] - currY) * math.cos(currA)
                deltaA = math.atan2(kekY, kekX)
                if distance < 15 or btn.any():
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
                    speed = Vmax * math.tanh(distance) * math.cos(deltaA)
                    deltaspeed = Kw * deltaA + Vmax * math.tanh(distance) * math.sin(2 * deltaA) / (2 * distance)
                    mL.run_direct(duty_cycle_sp=filter(speed - deltaspeed))
                    mR.run_direct(duty_cycle_sp=filter(speed + deltaspeed))
                    if getM[i]:
                        if not flag_down:
                            mM.run_direct(duty_cycle_sp=100)
                            timer = time.time()
                            flag_down = True
                        if flag_down:
                            if time.time() - timer > 1:
                                mM.stop(stop_action='brake')
                                flag_down = False
                    prefmL = currmL
                    prefmR = currmR
                    prefA = currA
                    prefX = currX
                    prefY = currY
                    # fh.write(str(currX) + ' ' + str(currY) + '\n')
                    fh.write('voltage_left: ' + str(speed + deltaspeed) + ', voltage_right: ' + str(
                        speed - deltaspeed) + '\n')
        finally:
            mL.stop(stop_action='brake')
            mR.stop(stop_action='brake')

'''
mA = LargeMotor('outC')
mB = LargeMotor('outA')
mA.position = current_time = start_time = 0

kps = 10
kpr = 10


def calculate_to_point(robot_x, robot_y, robot_angle, dist_x, dist_y):

    azimut = math.atan2((dist_y - robot_y), (dist_x - robot_x))
    course_angle = azimut - robot_angle

    goal_vector = math.sqrt((dist_x - robot_x) ** 2 + (dist_y - robot_y)** 2)

    if abs(course_angle) > math.pi:
        course_angle = course_angle - math.copysign(1, course_angle) * 2 * math.pi

    us = kps * goal_vector
    ur = kpr * course_angle
    v1 = us + ur
    v2 = us - ur

    if v1 > 100 or v1 < -100:
        v1 = math.copysign(100, v1)
    if v2 > 100 or v2 < -100:
        v2 = math.copysign(100, v2)

    control = (v1, v2)
    return control
if __name__ == '__main__':
    u1,u2 = calculate_to_point(0,0,0,10,10)
    try:
        while True:
            mA.run_direct(duty_cycle_sp = -100)
            mB.run_direct(duty_cycle_sp= -70)
    finally:
        mA.stop(stop_action = "brake")
        mB.stop(stop_action="brake")
        '''

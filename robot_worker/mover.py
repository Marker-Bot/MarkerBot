#!/usr/bin/python3
from ev3dev.ev3 import *
import time
import math

getX = [500]
getY = [0]
Rad = 31.2
Base = 175.55

mL = LargeMotor('outB')
mR = LargeMotor('outA')
sL = LightSensor('in2')
sR = LightSensor('in1')
fh = open('data.txt', 'w')
fh.write("0 0" + '\n')
btn = Button()


def getmL():
	return(mL.position)


def getmR():
	return(mR.position)


def filter(sp):
	if abs(sp) > 100:
		sp = 100*abs(sp)/sp
	return(sp)



def minval():
	return(min(sL.value(), sR.value()))


def error():
	if minval() < 400:
		return((minval() - 400)*math.copysign(1, sR.value() - sL.value()))
	else:
		return(0)


currX = currY = currA = 0
prefX = prefY = prefA = 0
currmL = currmR = prefmL = prefmR = 0
mL.position = mR.position = summ = 0

Vmax = 50
Kw = 60
Kp = 0.05
mL.stop(stop_action = 'coast')
mR.stop(stop_action = 'coast')

timest = time.time()

for k in range (1):
	for i in range(len(getX)):
		try:
			while True:
				distance = math.sqrt(math.pow(currX - getX[i], 2) + math.pow(currY - getY[i], 2))
				kekX = (getX[i] - currX)*math.cos(currA) + (getY[i] - currY)*math.sin(currA)
				kekY = (currX - getX[i])*math.sin(currA) + (getY[i] - currY)*math.cos(currA)
				deltaA = math.atan2(kekY,kekX) + Kp*error()
				if distance < 15 or btn.any():
					break
				else:
					currmL = getmL()
					currmR = getmR()
					dmL = currmL - prefmL
					dmR = currmR - prefmR
					summ = math.radians((dmR + dmL)/2)*Rad
					currA = prefA + math.radians((dmR - dmL)*Rad/Base)
					currX = prefX + math.cos(currA)*summ
					currY = prefY + math.sin(currA)*summ
					speed = Vmax * math.tanh(distance) * math.cos(deltaA)
					deltaspeed = Kw*deltaA + Vmax*math.tanh(distance)*math.sin(2*deltaA)/(2*distance)
					mL.run_direct(duty_cycle_sp = filter(speed - deltaspeed))
					mR.run_direct(duty_cycle_sp = filter(speed + deltaspeed))
					prefmL = currmL
					prefmR = currmR
					prefA = currA
					prefX = currX
					prefY = currY
					fh.write(str(currX) + ' ' + str(currY) + '\n')
		finally:
			mL.stop(stop_action = 'brake')
			mR.stop(stop_action = 'brake')
			time.sleep(0.1)
		fh.close

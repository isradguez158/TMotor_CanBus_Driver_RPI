import RPi.GPIO as GPIO
import can
import time
import os
import numpy as np
import datetime as dt
import TmotorCAN as TmotorCAN
import datetime
import math
import Plot_Isra as Plot_Isra
import matplotlib.pyplot as plt

date = datetime.datetime.now()

M1 = TmotorCAN.TmotorCAN(1)
plot = Plot_Isra.Plot_Isra()

start = time.time()
now = 0
t_pr1 = 0
t_pr2 = 0
Delta_T1 = .02
ref=0 

M1.Motor_stop()
time.sleep(0.1)
M1.Motor_stop()
time.sleep(0.1)
M1.Motor_run()
time.sleep(0.1)
M1.set_origin()
time.sleep(0.1)
M1.set_origin()
time.sleep(0.1)

M1.pos_control(0, 10, 1)
#time.sleep(0.1)

M1.decode()

print("actual: " + str(M1.pos*180/3.1416)+
		"    cmd: " + str(ref))
		
start = time.time()
now=(time.time()-start)

while(True):
	
	now=(time.time()-start)
	
	#ref=45*math.sin(now)

	if (now - t_pr2 > .005):
		t_pr2 = now
		ref=45*math.sin(now)
		M1.pos_control(ref, 10, 1)
		M1.decode()
		plot.update(now, ref, M1.pos*180/3.1416)
		plt.ioff()
		plt.show()
		#print("actual_pos: " + str(M1.pos*180/3.1416)+"    cmd_pos: " + str(ref))
	'''
		
	if (now - t_pr2 > .005):
		t_pr2 = now
		ref=1*math.sin(now)
		M1.torque_control(ref)
		M1.decode()
		print("actual_torque: " + str(M1.torque)+
		"    cmd_torque: " + str(ref))
'''

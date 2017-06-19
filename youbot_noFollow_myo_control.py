#!/usr/bin/python
''' DON'T TOUCH THESE FIRST LINES! '''
''' ============================== '''
from PyoConnect import *
import numpy as np
from youbotControl.ArmPos import *
from elbow_force_predict import *
from collections import deque
import time
EMG_data = deque()
head_move = 0
flag = 0
activition = 0
myo = Myo(sys.argv[1] if len(sys.argv) >= 2 else None) 

''' ============================== '''

''' OK, edit below to make your own fancy script ^.^ '''

# Edit here:
def onEMG(emg,state):
	global handMusle
	global activition
	global EMG_data
	if len(EMG_data) >= 30:
		EMG_data.append(sum(emg)*0.8)
		EMG_data.popleft()
	else:
		EMG_data.append(sum(emg)*0.8)
	
def onLock():
	global flag	
	flag = 1
	if flag>=1 :
		myo.unlock('hold')
	
def esti_wrist():
	wrist = 0	
	if myo.getPose() == "waveIn":
		wrist = -1
	if myo.getPose() == "waveOut":
		wrist = 1
	return wrist

def grasp_estimate(activition):
	if myo.getPose() == "fist" and activition >= 0.6:
		grasp = 0.001
	else:
		grasp =0.011
	return grasp

def protect_wrist(angle_move):
	angle = angle_move	
	if angle_move >= 1.7:
		angle = 1.7
	if angle_move < 0:
		angle =0.01
	return angle

def roll_estimat(ud):
	rollmove = (myo.getRoll()+ud)*180/np.pi	
	if rollmove > 50:
		rollmove = 50
	if rollmove < 25:
		rollmove =0
	return rollmove*0.05
		
	
def onPeriodic():
	global activition
	global EMG_data
	global head_move
	global Arm
	global step
	global flag
	global mode
	global pub
	if myo.getPose() != 'unknown' and myo.isUnlocked():
		pose = myo.getPose()
		handMusle.calculate_activition(EMG_data,0.0005,-3)	
		print(pose)
		angleUD = myo.getPitch()/np.pi*180
		angleLR = myo.getYaw()/np.pi*180
		if flag == 0:
			step = angleLR
			flag += 1
		
		if (angleLR-step) < 0:
			angleLR = 360 + angleLR-step
		else:
			angleLR = angleLR-step
		
		
		ud = myo.getPitch()
		if ud > 0:
			ud = -0.5*ud
		
		head_move += esti_wrist()*0.005	
		angle1 = 0.016111*angleLR
		angle2 = 0.6
		angle3 = (-angleUD+90)*0.013889-4.5
		angle4 = protect_wrist(0.8+head_move)
		angle5 = 3.0#roll_estimat(ud)
		grasp = grasp_estimate(handMusle.activition)
		
		print(angle1,angle2,angle3,angle4,angle5,grasp)
		with open("main.txt","w") as text_file:
			text_file.write("{0}\n{1}\n{2}\n{3}\n{4}\n{5}\n{6}".format(angle1,angle2,angle3,angle4,angle5,grasp,grasp))
		#Arm.moveArm([angle1,angle2,angle3,angle4,angle5])
		#Arm.moveGripper([grasp,grasp])
		

# Stop editting

# Comment out below the events you are not using
myo.onLock = onLock
#myo.onUnlock = onUnlock
#myo.onPoseEdge = onPoseEdge
myo.onPeriodic = onPeriodic
#myo.onWear = onWear
#myo.onUnwear = onUnwear
myo.onEMG = onEMG
#myo.onBoxChange = onBoxChange

''' DON'T TOUCH BELOW THIS LINE! '''
''' ============================ '''
myo.connect()
step = 0.0
flag = 0
handMusle = Muscle_Tendon(np.array([0.031,0.30,0,1]),np.array([0.027,-0.03,0, 1]))
#Arm = ArmPos()
try:
	while(1):
		myo.run()
		myo.tick()
except:
	print e
finally:
	myo.tick()


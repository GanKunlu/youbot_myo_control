#!/usr/bin/python
''' DON'T TOUCH THESE FIRST LINES! '''
''' ============================== '''
from PyoConnect import *
import numpy as np
from youbotControl.ArmPos import *
from elbow_force_predict import *
from collections import deque
import time
import rospy
import math
from geometry_msgs.msg import Twist
EMG_data = deque()
head_move = 0
flag = 0
mode = "base"
bFlag = False
moveBindings = {
		'fingersSpread':(1,0), 	# forwards
		'waveIn':(0,1.3), 	# left
		'waveOut':(0,-0.1),	# right
		'fist':(-1,0), 	# backward
	       }
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
	global moveBindings
	global bFlag
	if myo.getPose() != 'unknown':
		pose = myo.getPose()
		handMusle.calculate_activition(EMG_data,0.0005,-3)	
 
		if(myo.getPose() == 'doubleTap' and bFlag == False):
			bFlag = True
			print("connect!")
		if bFlag == True:
						
			if mode == "base":
				mode = "arm"
			else:
				mode = "base"
			bFlag = False
			

		if mode == "arm":
			x = 0
			th = 0
			angleUD = myo.getPitch()/np.pi*180
			angleLR = myo.getYaw()/np.pi*180
			if flag == 0:
				step = angleLR
				flag += 1
		
			if (angleLR-step) < 0:
				angleLR = 360 + angleLR-step
			else:
				angleLR = angleLR-step
		
			'''
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
			'''
			x = 0.40 #0.457888
			z = 0.42+0.2*math.tan(-angleUD/180.0*np.pi) # 0.346427
			y = 0.1*math.sin(-angleLR/180.0*np.pi) # -0.00760936
			print(x,y,z)
			with open("picture.txt","w") as text_file:
				text_file.write("{:.3f}\n{:.3f}\n{:.3f}\n".format(x,y,z))
			'''
			print(angle1,angle2,angle3,angle4,angle5,grasp)
			with open("main.txt","w") as text_file:
				text_file.write("{0}\n{1}\n{2}\n{3}\n{4}\n{5}\n{6}".format(angle1,angle2,angle3,angle4,angle5,grasp,grasp))
			'''
			twist = Twist()
			twist.linear.x = 0 
			twist.linear.y = 0
			twist.linear.z = 0
			#print(pose,turn,speed)
			twist.angular.x = 0 
			twist.angular.y = 0
			twist.angular.z = 0
			#pub.publish(twist)

		if mode == "base":
			rollmove = (myo.getRoll())*180/np.pi
			pose = myo.getPose()
			if pose in moveBindings.keys():
				x = moveBindings[pose][0]
				th = moveBindings[pose][1]+0.015*rollmove
			else:
				x = 0
				th = 0
			act = handMusle.activition
			if act < 0.4:
				act = 0.4
			if act > 0.6:
				act = act*1.1
			if act > 0.7:
				act = act*1.1
			if act >0.75:
				act = act*1.1
			if act > 0.8:
				act = act*1.1
			if act >0.85:
				act = act*1.1
			if act > 0.9:
				act = act*1.2
			speed = 0.13*(act-0.4)+0.02		
			turn = 3*speed
			twist = Twist()
			twist.linear.x = x*speed 
			twist.linear.y = 0
			twist.linear.z = 0
			#print(pose,turn,speed)
			twist.angular.x = 0 
			twist.angular.y = 0
			twist.angular.z = th*turn
			#pub.publish(twist)

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
#rospy.init_node('youbot_teleop')
#pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
#Arm = ArmPos()
try:
	while(1):
		myo.run()
		myo.tick()
except:
	print e
finally:
	twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        #pub.publish(twist)
	myo.tick()


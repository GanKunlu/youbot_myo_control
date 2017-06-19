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
from geometry_msgs.msg import Twist
EMG_data = deque()
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
		
	
def onPeriodic():
	global EMG_data
	global moveBindings
	global handMusle
	global pub
	if myo.getPose() != 'unknown' and myo.isUnlocked():
		pose = myo.getPose()
		handMusle.calculate_activition(EMG_data,0.0005,-3)
		rollmove = (myo.getRoll())*180/np.pi
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
		speed = 0.18*(act-0.4)+0.02		
		turn = 3*speed
	
		twist = Twist()
		twist.linear.x = x*speed 
		twist.linear.y = 0
		twist.linear.z = 0
		#print(pose,turn,speed)
		twist.angular.x = 0 
		twist.angular.y = 0
		twist.angular.z = th*turn
		pub.publish(twist)
		
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
rospy.init_node('youbot_teleop')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

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
        pub.publish(twist)

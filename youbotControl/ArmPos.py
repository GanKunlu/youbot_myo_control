# @author: Gan  Date:2016/12/28

import sys
import copy
import rospy
import math
from brics_actuator.msg import JointPositions, JointValue
from std_msgs.msg import String
import geometry_msgs.msg
import control_msgs.msg
import actionlib
from std_msgs.msg import String

class Arm(object):
	'''
	
	'''
	def __init__(self, isMoveit = False):
		self._joint_uri = ['arm_joint_1','arm_joint_2','arm_joint_3','arm_joint_4','arm_joint_5','gripper_finger_joint_l','gripper_finger_joint_r']
		self.jointValueList = []
		self._jointInit()
		self._jpArm = JointPositions()
		self._jpGripper = JointPositions()
		self.isMoveit = isMoveit  
		rospy.init_node('youbot_exec')
		self.rate = rospy.Rate(10)

	def _jointInit(self):
		for joint_i, uri in enumerate(self._joint_uri):
			stepJointval = JointValue()
			self.jointValueList.append(copy.deepcopy(stepJointval))
			self.jointValueList[joint_i].joint_uri = uri

	def moveArm(self, Position):
		assert len(Position) == 5, "Position value numbers must be 5, or not matched !"
		for i in range(5):
			self.jointValueList[i].unit = 'rad'
			self.jointValueList[i].value = Position[i]
		self._jpArm.positions = self.jointValueList
		self.publish("arm_1/arm_controller/position_command", JointPositions, 5, self._jpArm)
		rospy.loginfo("moving arms...")
		

	def moveGripper(self, Position):
		assert len(Position) == 2, "Gripper Position numbers must be 2 !"
		for i in range(2):
			self.jointValueList[5+i].unit = 'm'
			self.jointValueList[5+i].value = Position[i]
		self._jpGripper.positions = self.jointValueList[5:7]
		self.publish("arm_1/gripper_controller/position_command", JointPositions, 2, self._jpGripper)
		rospy.loginfo("moving grippers...")
		
	def publish(self,topic,msg,size,jp):
		publisher = rospy.Publisher(topic, msg, queue_size = size)
		publisher.publish(jp)
		self.rate.sleep()
		
		
		#print("move arm Position to"+str(Position))
	
	
			
			
		
		

# @author: Gan  Date:2016/12/30

import sys
import copy
import rospy
import math
from brics_actuator.msg import JointPositions, JointValue
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler #, quaternion_about_axis, quaternion_from_matrix, rotation_matrix
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import control_msgs.msg
import actionlib
from std_msgs.msg import String

class ArmTraj(object):
	'''
	
	'''
	def __init__(self):
		self._joint_uri = ['gripper_finger_joint_l','gripper_finger_joint_r']
		moveit_commander.roscpp_initialize(sys.argv)
		self.robotCommander = moveit_commander.RobotCommander()
 		scene = moveit_commander.PlanningSceneInterface()
		print "Creating move group commander object"
		self.group = moveit_commander.MoveGroupCommander("manipulator")
		self.group.set_planning_time(15)
		self.arm_client = actionlib.SimpleActionClient('/arm_1/arm_controller/follow_joint_trajectory', 
                                        control_msgs.msg.FollowJointTrajectoryAction)		
		self._gripperValueList = []		
		self.Poses =[]
		rospy.init_node('youbot_traject', anonymous = False) 
		self._jointInit()
		self._jpGripper = JointPositions()  
		#self.rate = rospy.Rate(5)
		
	def _jointInit(self):
		for joint_i, uri in enumerate(self._joint_uri):
			stepJointval = JointValue()
			self._gripperValueList.append(copy.deepcopy(stepJointval))
			self._gripperValueList[joint_i].joint_uri = uri		
		rospy.loginfo("Create arm joint trajectory action client " + '/arm_1/arm_controller/follow_joint_trajectory')
		rospy.loginfo("waiting for arm action server")
		self.arm_client.wait_for_server()
		print "============ Planning reference frame: %s" % self.group.get_planning_frame()
		print "============ End effector link: %s" % self.group.get_end_effector_link()
		print "============ Robot Groups: %s" % self.robotCommander.get_group_names()

	def addPose(self, Position,Angle):
		assert len(Position) == 3, "Position value numbers must be 3, or not matched !"
		assert len(Angle) == 3, "Oriention value numbers must be 3, or not matched !"
		
		self.group.set_pose_reference_frame("base_link")
		Pose = geometry_msgs.msg.PoseStamped()
		quat = quaternion_from_euler(Angle[0],Angle[1],Angle[2])
		Pose.pose.position.x = Position[0]
		Pose.pose.position.y = Position[1]
		Pose.pose.position.z = Position[2]
		Pose.pose.orientation.x = quat[0]
		Pose.pose.orientation.y = quat[1]
		Pose.pose.orientation.z = quat[2]
		Pose.pose.orientation.w = quat[3]
		self.Poses.append(copy.deepcopy(Pose.pose))

	def moveArm(self):
		assert len(self.Poses) > 1, "The number of Poses is too less!"
		self.group.clear_pose_targets()
		self.group.set_pose_targets(self.Poses,"gripper_pointer_link")
		plan = self.group.plan()
		if len(plan.joint_trajectory.points) == 0:
			rospy.loginfo("Trajectory not found")
		else:
			plan.joint_trajectory = self.trajectory_toffset(plan.joint_trajectory, 0)
			rospy.loginfo("Moving to pose")
		goal = control_msgs.msg.FollowJointTrajectoryGoal()
		goal.trajectory = copy.deepcopy(plan.joint_trajectory)
		goal.trajectory = self.massage_traj(goal.trajectory)
		#self.arm_client.send_goal(goal, feedback_cb = self.jtg_feedback_cb)
		#self.arm_client.wait_for_result()
		#return self.arm_client.get_result()
	
	def showAllPoses(self):
		for i, Pose in enumerate(self.Poses):
			print "pose",i,":"
			print "    positions: x,",Pose.position.x,"  y,",\
				Pose.position.y,"  z,",Pose.position.z
			print "    oritentation: x",Pose.orientation.x,"  y,",\
				Pose.orientation.y,"  z,",Pose.orientation.z,\
				"  w,",Pose.orientation.w
			print ""

	def delPoses(self,delList = []):
		if len(delList) == 0:
			self.Poses = [] 
		else:
			for pose in delList:
				del self.Poses[pose]
		

	def jtg_feedback_cb(self, data):  
		rospy.loginfo('arm is moving')
		rospy.loginfo(data)
	
	def trajectory_toffset(self,traj, tsecs):
		for ii in range(0,len(traj.points)):
			traj.points[ii].time_from_start.secs+= tsecs
			return traj		
	
	def massage_traj(self,traj_in):
		traj_out = JointTrajectory()
		traj_out.joint_names = traj_in.joint_names
		npts = len(traj_in.points)

		# take first point
		traj_out.points.append(copy.deepcopy(traj_in.points[0]))
		traj_out.points[0].velocities = [0,0,0,0,0]
		traj_out.points[0].accelerations = [0,0,0,0,0]

		# take last point
		traj_out.points.append(copy.deepcopy(traj_in.points[-1]))
		traj_out.points[-1].velocities = [0,0,0,0,0]
		traj_out.points[-1].accelerations = [0,0,0,0,0]

		print "Points in traj: " + str(traj_out.points)
		return traj_out
	
	def moveGripper(self, Position):
		assert len(Position) == 2, "Gripper Position numbers must be 2 !"
		for i in range(2):
			self._gripperValueList
			self._gripperValueList[i].unit = 'm'
			self._gripperValueList[i].value = Position[i]
		self._jpGripper.positions = self._gripperValueList
		self.publish("arm_1/gripper_controller/position_command", JointPositions, self._jpGripper)
		self.loginfo("moving grippers...")
		
	def publish(self,topic,msg,jp):
		publisher = rospy.Publisher(topic, msg, latch=True)
		publisher.publish(jp)
		self.rate.sleep()
		#print("move arm Position to"+str(Position))
	
			
			
		
		

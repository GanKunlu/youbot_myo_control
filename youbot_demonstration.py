#!/usr/bin/python
''' DON'T TOUCH THESE FIRST LINES! '''
''' ============================== '''
from PyoConnect import *
import numpy as np
from elbow_force_predict import *
from collections import deque
import time
import rospy
from geometry_msgs.msg import Twist
import keylogger
import time
import threading
import os
from gmr_optimization import *

t_s = 100
isin_demonstration = False
isin_reproduction = False
ges_flag = False
isMove = True
isGMR = False
EMG_data = deque()
head_move = 0
flag = 0
mode = "base"
bFlag = False
gripAngle = 0
moveBindings = {
    'fingersSpread': (1, 0),  # forwards
    'waveIn': (0, 1.3),  # left
    'waveOut': (0, -0.1),  # right
    'fist': (-1, 0),  # backward
}
activition = 0
start_time = time.time()
current_time = time.time()
isStart = False
myo = Myo(sys.argv[1] if len(sys.argv) >= 2 else None)

''' ============================== '''

''' OK, edit below to make your own fancy script ^.^ '''


# Edit here:
def onEMG(emg, state):
    global handMusle
    global activition
    global EMG_data
    if len(EMG_data) >= 30:
        EMG_data.append(sum(emg) * 0.8)
        EMG_data.popleft()
    else:
        EMG_data.append(sum(emg) * 0.8)


def onLock():
    global flag
    flag = 1
    if flag >= 1:
        myo.unlock('hold')


def esti_wrist():
    wrist = 0
    if myo.getPose() == "waveIn":
        wrist = -1
    if myo.getPose() == "waveOut":
        wrist = 1
    return wrist


def grasp_estimate(activition):
    if myo.getPose() == "fist" and activition >= 0.55:
        grasp = 0.001
    else:
        grasp = 0.011
    return grasp


def protect_wrist(angle_move):
    angle = angle_move
    #1.7
    if angle_move >= 1.7:
        angle = 1.7
    if angle_move < 0:
        angle = 0.01
    return angle


def roll_estimat(ud):
    rollmove = (myo.getRoll() + ud) * 180 / np.pi
    if rollmove > 50:
        rollmove = 50
    if rollmove < 25:
        rollmove = 0
    return rollmove * 0.05


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
    global gripAngle
    global isin_demonstration
    global isin_reproduction
    global start_time
    global current_time
    global isStart
    global isMove
    if myo.getPose() != 'unknown' and isin_reproduction == False and isMove:
        if isStart == False:
            print("All is right, youbot is start!")
            isStart = True
        handMusle.calculate_activition(EMG_data, 0.0005, -3)
        if (myo.getPose() == 'doubleTap' and bFlag == False):
            bFlag = True

        if bFlag == True:
            if mode == "base":
                mode = "arm"
            else:
                mode = "base"
            bFlag = False

        if mode == "arm":

            angleUD = myo.getPitch() / np.pi * 180
            angleLR = myo.getYaw() / np.pi * 180
            if flag == 0:
                step = angleLR
                flag += 1

            if (angleLR - step) < 0:
                angleLR = 360 + angleLR - step
            else:
                angleLR = angleLR - step

            if (angleUD > 49 and gripAngle == 0):
                gripAngle = angleLR
            if (angleUD <= 49):
                gripAngle = angleLR


            head_move += esti_wrist() * 0.005
            angle1 = 0.016111 * gripAngle
            #angle2 = 0.45 + (-angleUD) * 0.0026
            #angle3 = (-angleUD + 90) * 0.013889 - 3.5
            #angle4 = protect_wrist(0.8 + (-angleUD + 90) * 0.003)
            angle2 = 1.0+ (-angleUD) * 0.0036
            angle3 = (-angleUD + 90) * 0.015889 - 4.5
            angle4 = protect_wrist(0.8 + (-angleUD + 90) * 0.003)
            angle5 = 3.12 - 0.008 * (-myo.getRoll()) * 180 / np.pi
            grasp = grasp_estimate(handMusle.activition)
            if(isin_reproduction == False):
                with open("main.txt", "w") as text_file:
                    text_file.write("{0}\n{1}\n{2}\n{3}\n{4}\n{5}\n{6}".format(angle1, angle2, angle3, angle4, angle5, grasp, grasp))
                    #rospy.loginfo("moving arms...")

            if (isin_demonstration == True):
                current_time = time.time()
                if(current_time-start_time > 1.0/t_s):
                    with open("demon_file.txt", "a") as demon_file:
                        demon_file.write(
                            "{0} {1} {2} {3} {4} {5} {6}\n".format(angle1, angle2, angle3, angle4, angle5, grasp, grasp))
                    start_time = time.time()

            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            pub.publish(twist)

        if mode == "base":
            rospy.loginfo("moving base...")
            rollmove = (myo.getRoll()) * 180 / np.pi
            pose = myo.getPose()
            if pose in moveBindings.keys():
                x = moveBindings[pose][0]
                th = moveBindings[pose][1] + 0.015 * rollmove
            else:
                x = 0
                th = 0
            act = handMusle.activition
            if act < 0.4:
                act = 0.4
            if act > 0.6:
                act = act * 1.1
            if act > 0.7:
                act = act * 1.1
            if act > 0.75:
                act = act * 1.1
            if act > 0.8:
                act = act * 1.1
            if act > 0.85:
                act = act * 1.1
            if act > 0.9:
                act = act * 1.2
            speed = 0.13 * (act - 0.4) + 0.02
            turn = 3 * speed
            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            pub.publish(twist)




def onKeyboardEvent(t, modifiers, keys):
    global isin_demonstration
    global isin_reproduction
    global isMove
    global isGMR
    if keys == "<enter>" and isin_demonstration == False:
        print("# Start demonstration!")
        isin_demonstration = True
        open("demon_file.txt","w")
    if keys == "<esc>" and isin_demonstration == True:
        print("# Demonstration end! press key 's' to start reproduction...")
        isin_demonstration = False
    if keys == "s" and isin_demonstration == False and isin_reproduction == False and os.path.isfile('demon_file.txt'):
        print("# Start reproduction with normal optimization! ")
        isin_reproduction = True
    if keys == "o" and isin_demonstration == False and isin_reproduction == False and os.path.isfile('demon_file.txt'):
        print("# Start reproduction with GMR optimization! ")
        isin_reproduction = True
        isGMR = True

    if keys == "m" and isin_reproduction == False:
        print("# recover moving! ")
        isMove = True

def reproduction():
    global isin_reproduction
    global isMove
    global t_s
    global ges_flag
    global isGMR
    while(True):
        #time.sleep(2)
        if(isin_reproduction == True):
            if(isGMR):
                dem_opter = gmr_from_file([8, 5], "demon_file.txt")
                dem_opter.do_gmr(is_plot = False, n_iter=5)
                dem_opter.save_to_file()
                with open("demon_file1.txt","r") as demon_file:
                    lines = demon_file.readlines()
                for line in lines:
                    line = line.strip()
                    listFromLine = line.split(" ")
                    with open("main.txt","w") as main_file:
                        main_file.write("\n".join(listFromLine))
                    if (listFromLine[-1] == "0.001" and ges_flag == False):
                        ges_flag = True
                        time.sleep(1)
                    else:
                        time.sleep(0.008)
                    if(listFromLine[-1] == "0.011"):
                        ges_flag = False
                isin_reproduction = False
                isMove = False
                isGMR = False
                print("done!")

            else:
                with open("demon_file.txt","r") as demon_file:
                    lines = demon_file.readlines()
                for line in lines:
                    line = line.strip()
                    listFromLine = line.split(" ")
                    with open("main.txt","w") as main_file:
                        main_file.write("\n".join(listFromLine))
                    if (listFromLine[-1] == "0.001" and ges_flag == False):
                        ges_flag = True
                        time.sleep(1)
                    else:
                        time.sleep(0.008)
                    if(listFromLine[-1] == "0.011"):
                        ges_flag = False
                isin_reproduction = False
                isMove = False
                print("done!")

def getkeyboard():
    global done
    now = time.time()
    done = lambda: time.time() > now + 20000
    keylogger.log(done, onKeyboardEvent)

# Stop editting

# Comment out below the events you are not using
myo.onLock = onLock
# myo.onUnlock = onUnlock
# myo.onPoseEdge = onPoseEdge
myo.onPeriodic = onPeriodic
# myo.onWear = onWear
# myo.onUnwear = onUnwear
myo.onEMG = onEMG
# myo.onBoxChange = onBoxChange

''' DON'T TOUCH BELOW THIS LINE! '''
''' ============================ '''
myo.connect()
step = 0.0
flag = 0
handMusle = Muscle_Tendon(np.array([0.031, 0.30, 0, 1]), np.array([0.027, -0.03, 0, 1]))
rospy.init_node('youbot_teleop')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)


try:
    thread_1 = threading.Thread(target=getkeyboard)
    thread_1.setDaemon(True)
    thread_1.start()
    thread_2 = threading.Thread(target=reproduction)
    thread_2.setDaemon(True)
    thread_2.start()

    while (1):
        myo.run()
        myo.tick()

except:
    print e

finally:
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
    myo.tick()



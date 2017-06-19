from youbotControl.ArmPos import *
arm = Arm()
while(1):
	f = open('main.txt')
	msg = f.readlines()
	data = [0,0,0,0,0,0,0]
	for i,strings in enumerate(msg):
		data[i] = float(strings.strip())
	arm.moveArm(data[0:5])
	arm.moveGripper(data[5:7])
		

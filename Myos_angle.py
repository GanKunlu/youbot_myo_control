
import time
import numpy as np

	
def GetDegreeFromACC(ACCs):
	R = 0
	angle=[]
	ACC_angle = []
	for ACC in ACCs:
		R = 0
		for data in ACC:
			R += data**2
		R = np.sqrt(R)
		ACC_angle = []
		for data in ACC:
			ACC_angle.append(np.arccos(data/R)*180/np.pi)
		ACC_angle[2] = np.sign(ACC_angle[2])*(abs(ACC_angle[2])-abs(90-abs(ACC_angle[1])))
		angle.append(ACC_angle*np.sign([-ACC[1],-ACC[1],-ACC[0]]))
	return angle

def angle_init(myo,angle,which_myo):
	index =0
	for _one_myo in which_myo:
		angle[index]= GetDegreeFromACC([myo[_one_myo].acceleration])[0][2]
		index +=1
	return angle

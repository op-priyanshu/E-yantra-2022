'''
*****************************************************************************************
*
*        =================================================
*             Pharma Bot Theme (eYRC 2022-23)
*        =================================================
*                                                         
*  This script is intended for implementation of Task 2B   
*  of Pharma Bot (PB) Theme (eYRC 2022-23).
*
*  Filename:			task_2b.py
*  Created:				
*  Last Modified:		8/10/2022
*  Author:				e-Yantra Team
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:			task_2b.py
# Functions:		control_logic, read_qr_code
# 					[ Comma separated list of functions in this file ]
# Global variables:	
# 					[ List of global variables defined in this file ]

####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
##############################################################
import  sys
import traceback
import time
import os
import math
from zmqRemoteApi import RemoteAPIClient
import zmq
import numpy as np
import cv2
import random
from pyzbar.pyzbar import decode
##############################################################

################# ADD UTILITY FUNCTIONS HERE #################





##############################################################

def control_logic(sim):
	"""
	Purpose:
	---
	This function should implement the control logic for the given problem statement
	You are required to make the robot follow the line to cover all the checkpoints
	and deliver packages at the correct locations.

	Input Arguments:
	---
	`sim`    :   [ object ]
		ZeroMQ RemoteAPI object

	Returns:
	---
	None

	Example call:
	---
	control_logic(sim)
	"""
	##############  ADD YOUR CODE HERE  ##############
	
	
	rj=sim.getObject("/right_joint")
	lj=sim.getObject("/left_joint")
	vs=sim.getObject("/vision_sensor")

	sim.setJointTargetVelocity(lj,2)
	sim.setJointTargetVelocity(rj,2)

	while True:
		t=sim.getSimulationTime()
		img,res=sim.getVisionSensorImg(vs)
		img = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
		linecolorset=0
		frame_findline = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		retval,frame_findline = cv2.threshold(frame_findline , 0, 255, cv2.THRESH_OTSU)
		frame_findline = cv2.erode(frame_findline, None, iterations=6)
		
		colorpos1 = frame_findline[1]
		colorpos2 = frame_findline[510]
		
		try:
			lineColorCount_Pos1 = np.sum(colorpos1 == linecolorset)
			lineColorCount_Pos2 = np.sum(colorpos2 == linecolorset)

			lineIndex_pos1 = np.where(colorpos1 == linecolorset)
			lineIndex_pos2 = np.where(colorpos2 == linecolorset)
			if lineColorCount_Pos1 == 0:
				lineColorCount_Pos1 = 1
			if lineColorCount_Pos2 == 0:
				lineColorCount_Pos2 = 1
			left_pos1 = lineIndex_pos1[0][lineColorCount_Pos1-1]
			right_pos1 = lineIndex_pos1[0][0]
			center_pos1 = int((left_pos1+right_pos1)/2)

			left_pos2 = lineIndex_pos2[0][lineColorCount_Pos2-1]
			right_pos2 = lineIndex_pos2[0][0]
			center_pos2 = int((left_pos2+right_pos2)/2)
			
			center = int((center_Pos1 + center_pos2)/2)
		except:
			center = None
			pass
		print(center)
		
			

		


		cv2.imshow('Frame',frame_findline)
		if cv2.waitKey(1) & 0xFF==ord('d'):
			break


	capture.release()
	cv2.destroyAllWindows()

	'''
	
	count=0
	while True:

		
		t=sim.getSimulationTime()
		
		result,data,d2=sim.readVisionSensor(vs)
		print(data[3],data[9],data[10],data[11],data[12],data[13])
		image,resolution=sim.getVisionSensorImg(vs,)
		
		
            
		image( image)	
		cv2.waitKey(0)    
		cv2.destroyAllWindows()

		
		if(data[3]<0.2):

			while True:
				result,dat,d2=sim.readVisionSensor(vs)
				
				if(dat[10]>0.7 and dat[11]>0.7 and dat[12]>0.7):
					while True:
						print('wtf')
						result,da,d2=sim.readVisionSensor(vs)
						print(da[3],da[4],da[5],da[6],da[7],da[8])
						
						t2=sim.getSimulationTime()
						
						if abs(t2-t)>=9:
							print('khatam')
							sim.setJointTargetVelocity(lj,3)
							sim.setJointTargetVelocity(rj,3)
							if count==0:
								count=1
							else :
								count=0
							break
						if count==0:
							sim.setJointTargetVelocity(lj,-.95) 
							sim.setJointTargetVelocity(rj,.95)
					
		        				
			
						else :
							sim.setJointTargetVelocity(lj,.95) 
							sim.setJointTargetVelocity(rj,-.95)
					break
		
	'''
		

			
				
		        	
				
		
		
					
			
	
	
				

	
	##################################################

def read_qr_code(sim):
	"""
	Purpose:
	---
	This function detects the QR code present in the camera's field of view and
	returns the message encoded into it.

	Input Arguments:
	---
	`sim`    :   [ object ]
		ZeroMQ RemoteAPI object

	Returns:
	---
	`qr_message`   :    [ string ]
		QR message retrieved from reading QR code

	Example call:
	---
	control_logic(sim)
	"""
	qr_message = None
	##############  ADD YOUR CODE HERE  ##############
	vs=sim.getObject("/vision_sensor")
	img, resX, resY = sim.getVisionSensorCharImage(vs)
	img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
	barc=decode(img)
	qr_message = barc.data.decode("utf-8")
	##################################################
	return qr_message


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THE MAIN CODE BELOW #########

if __name__ == "__main__":
	client = RemoteAPIClient()
	sim = client.getObject('sim')	

	try:

		## Start the simulation using ZeroMQ RemoteAPI
		try:
			return_code = sim.startSimulation()
			if sim.getSimulationState() != sim.simulation_stopped:
				print('\nSimulation started correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be started correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be started !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit()

		## Runs the robot navigation logic written by participants
		try:
			
			control_logic(sim)
			time.sleep(5)
		except Exception:
			print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually if required.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

		
		## Stop the simulation using ZeroMQ RemoteAPI
		try:
			return_code = sim.stopSimulation()
			time.sleep(0.5)
			if sim.getSimulationState() == sim.simulation_stopped:
				print('\nSimulation stopped correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be stopped correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be stopped !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit()

	except KeyboardInterrupt:
		## Stop the simulation using ZeroMQ RemoteAPI
		return_code = sim.stopSimulation()
		time.sleep(0.5)
		if sim.getSimulationState() == sim.simulation_stopped:
			print('\nSimulation interrupted by user in CoppeliaSim.')
		else:
			print('\nSimulation could not be interrupted. Stop the simulation manually .')
			sys.exit()
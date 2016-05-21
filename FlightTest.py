import os
import curses
import subprocess
import serial
import cv2
import time
import struct
import array
import threading
import Queue
import numpy as np
import math
import BasicFns
import NavFns
import ImageProcessing
import AutoB4Fbwb
import shutil
#/home/pi/FlightTest/
TrialNo = 1
totTrials = 1
HiL = True
ImgPro = True
RPi = False
cd2r = math.pi/18000
d2r = math.pi/180
r2d = 180/math.pi
r2cd = 18000/math.pi
AutoWpTh = 10				#180			#For Mallala and Woomera Tests
#AutoWpTh =	200			#For Mawson Lakes Tests
RollCdMax = 4500
ex = 'auto' 		#'1e-8'
awb = 'auto' 		#'sun'
photo_width  = 800
photo_height = 600
TiOutTh = 2

#Data1 = np.load('/home/pi/FlightTest/Database/D1.npy')
#Data2 = np.load('/home/pi/FlightTest/Database/D2.npy')
#Data3 = np.load('/home/pi/FlightTest/Database/D3.npy')
#HeadErr = np.loadtxt('/home/pi/FlightTest/Database/HeadErr1.txt')
HeadErr = 0
Data1 = np.load('Database/Wp1Data.npy')
Data2 = np.load('Database/Wp2Data.npy')
Data3 = np.load('Database/Wp3Data.npy')

q_IpCalDrift = Queue.Queue()
q_POS = Queue.Queue()
q_EUANGS = Queue.Queue()
q_VEL = Queue.Queue()
q_GPSSTAT = Queue.Queue()
q_ti = Queue.Queue()
q_MainCounter = Queue.Queue()
q_POSatVisWp = Queue.Queue()
q_CAng = Queue.Queue()
q_TiOutFlag = Queue.Queue()
q_ImProFail = Queue.Queue()

AutoWps = [[-309353065,1365454102],[-309298649,1365464783],[-309315224,1365422668]]#Wo Auto Wps 28/4/15:0952 Wps 2,4 and 6 of WoomeraWps1
VisualWps = [[-344249458,1385246124],[-344184456,1385176544],[-344140892,1385357971]]#[[-344249458,1385246124],[-344249458,1385286124]]
#VisualWps = [[-344249458,1385246124],[-344222749,1385287571]]#[[-344249458,1385246124],[-344222832,1385262909]#[[-344249458,1385246124],[-344184456,1385176544],[-344140892,1385357971]]  
l_VisWps = len(VisualWps)

# Classes for Parallel Processing
class ImagProc:
	def IP(self,Data1,Data2,Data3,Yaw,WpNo,IpCalDrift,ImProFail,awb,ex,photo_width,photo_height,fn,stop_event,ImCapEvent,tiout_event,impro_event):
		#Image Processing Thresholds
		NLdmrk = 4							# Number of candidate Landmarks to be detected
		ImageTh = 0.1 					# Used in LdmrkDet.LD: To prevent excessive whites in binary image.
		BinaryTh = 130					# Initial binary threshold to  adaptively binarize an image
		BwAreaTh = [100,1500] 	#[200,3000]
		LdmrkAngTh = 5
		WpAngTh = 5
		WpSiRatTh = 0.1
		#End of Image Processing Thresholds
		IpCalDrift = np.array(2,dtype = 'float')	
		while (not tiout_event.is_set()) and (not stop_event.is_set()):
			if WpNo == 1:
				if RPi == True:
					filename = '/home/pi/FlightTest/FlightData/FBWB/TrialNo' + str(TrialNo) + '/Wp_' + str(WpNo) + '_Image' + awb + '.jpg'
					filename1 = '/home/pi/FlightTest/Database/photo_1_auto.jpg'
					cmd = 'raspistill -o ' + filename + ' -t 1000 -ss ' + str(ex) + ' -awb ' + awb + ' -w ' + str(photo_width) + ' -h ' + str(photo_height)
					pid = subprocess.call(cmd, shell=True)
				else:
					filename = 'Database/waypoint_'+str(WpNo)+ '_test.jpg'
				Im = cv2.imread(filename)
#				cv2.imshow('Image',Im)
#				cv2.waitKey(2000)
				Data = Data1
			elif WpNo == 2:
				if RPi == True:
					filename = '/home/pi/FlightTest/FlightData/FBWB/TrialNo' + str(TrialNo) + '/Wp_' + str(WpNo) + '_Image' + awb + '.jpg'
					filename1 = '/home/pi/FlightTest/Database/photo_2_auto.jpg'
					cmd = 'raspistill -o '+ filename + ' -t 1000 -ss ' + str(ex) + ' -awb ' + awb + ' -w ' + str(photo_width) + ' -h ' + str(photo_height)
					pid = subprocess.call(cmd, shell=True)
				else:
					filename = 'Database/waypoint_'+str(WpNo)+ '_test.jpg'
				Im = cv2.imread(filename)
				Data = Data2
			elif WpNo == 3:
				if RPi == True:
					filename = '/home/pi/FlightTest/FlightData/FBWB/TrialNo' + str(TrialNo) + '/Wp_' + str(WpNo) + '_Image' + awb + '.jpg'
					filename1 = '/home/pi/FlightTest/Database/photo_3_auto.jpg'
					cmd = 'raspistill -o ' + filename + ' -t 1000 -ss ' + str(ex) + ' -awb ' + awb + ' -w ' + str(photo_width) + ' -h ' + str(photo_height)
					pid = subprocess.call(cmd, shell=True)
				else:
					filename = 'Database/waypoint_'+str(WpNo)+ '_test.jpg'
				Im = cv2.imread(filename)
				Data = Data3
			fn.write('########## JUST AFTER IMAGE CAPTURE ##########' + '\n')		
			ImCapEvent.set()					
			st_ti_ImPro = time.time()
			while 1:
				try:
#					IpCalDrift = ImageProcessing.IP(Im,Data,Yaw,NLdmrk,ImageTh,BinaryTh,BwAreaTh,LdmrkAngTh,WpAngTh,WpSiRatTh)
#					IpCalDrift = ImageProcessing.IP(Im,Yaw,AltRef,YawRef,curWpNo,GenData,parLdmrkNos)
					ImageProcessing.LoadData(WpNo,False,'0')
					IpCalDrift = ImageProcessing.IP(Im,Yaw,0,0,WpNo,False,'0')  
					ImProTi = time.time() - st_ti_ImPro
					ImProFlag = 1
					print 'Overall Image Proceesing Time =',ImProTi
					break
				except:
					print 'Error in Image Processing'
					ImProFail = 1
					impro_event.set()
					break	
			print 'Broken 1'
			q_IpCalDrift.put(IpCalDrift)
			q_ImProFail.put(ImProFail)
			stop_event.set()
			print 'Broken 2'

class TravelDurIP: 
	def TDIP(self,GPSSTAT,POS,EUANGS,VEL,TVEC,HiL,MaintStart,MainCounter,CAng,WpCounter,ti,HeadErr,fn,fn1,ReqYaw,RollCd,WindM,WindD,ParThCounter,TiOutFlag,TiOutTh,X2,Y2,stop_event,ImCapEvent,tiout_event,impro_event):								
		tStart = time.time()
		YawErrPrev = 0		
		POSatVisWp = [0,0,0]
		while not stop_event.is_set():
			if impro_event.is_set():
				TiOutFlag = 1
			TVEC,GPSSTAT,POS,EUANGS,VEL,CAng,MainCounter,ti,tStart,YawErrPrev,ParThCounter,TiOutFlag = NavFns.SerialOpsMoveDurImPro(ser,header,GPSSTAT,POS,EUANGS,VEL,TVEC,HiL,MaintStart,MainCounter,CAng,
			ti,HeadErr,fn,ReqYaw,tStart,WindM,WindD,YawErrPrev,RollCd,X2,Y2,ParThCounter,TiOutFlag,TiOutTh)
			if TiOutFlag == 1:
				tiout_event.set()
				break
			if ImCapEvent.is_set():
				# Save Visual Wp Data
				fn1.write('########## Data at Visual Waypoint:' + str(l_VisWps + 2 - WpCounter) + '##########' + '\n')	
				GPSSTATatVisWp = GPSSTAT[len(GPSSTAT)-1]
				fn1.write('#### GPS_stats ####' + '\n')	
				fn1.write(' '.join(map(str,GPSSTATatVisWp)) + '\n')	
				POSatVisWp = POS[len(POS)-1]
				fn1.write('#### POSatVisWp ####' + '\n')	
				fn1.write(' '.join(map(str,POSatVisWp)) + '\n')	
				EUANGSatVisWp = EUANGS[len(EUANGS)-1]
				fn1.write('#### EUANGSatVisWp ####' + '\n')	
				fn1.write(' '.join(map(str,EUANGSatVisWp)) + '\n\n')	
				# End of save Visual Wp Data
				ImCapEvent.clear()	
		print 'broken 3'		
		q_POSatVisWp.put(POSatVisWp)
		q_POS.put(POS)
		q_EUANGS.put(EUANGS)
		q_VEL.put(VEL)
		q_GPSSTAT.put(GPSSTAT)
		q_ti.put(ti)
		q_MainCounter.put(MainCounter)
		q_CAng.put(CAng) 
		q_TiOutFlag.put(TiOutFlag)
		print 'broken 4'
# End of Classes for Parallel Processing			

###########################################################################################################################################################################
########################################################################## MAIN PROGRAM STARTS HERE #######################################################################
###########################################################################################################################################################################	
while TrialNo <= totTrials:
	print 'TrialNo = ',TrialNo
	if RPi == True:
		ser = serial.Serial('/dev/ttyUSB0',57600,timeout=0.5)
		print("Connected to: "+ser.portstr)
		ser.flushInput()
		ser.flushOutput()
		fn = open('/home/pi/FlightTest/FlightData/MainDataFile.txt','w+')
		fn1 = open('/home/pi/FlightTest/FlightData/OtherDataFile.txt','w+')
	else:
		ser = 0
		fn = open('FlightData/MainDataFile.txt','w+')
		fn1 = open('FlightData/OtherDataFile.txt','w+')				
	names  = ['MainCounter','time','pos_x','pox_y','pos_z','roll','pitch','yaw','lat','lng','alt','asp']
	fn.write('\t'.join(names) + '\n')
	fn1.write('Other Data' + '\n')	

	print("Initializaing")
	POS = [[0,0,400]]
	POS = np.array(POS)	
	TVEC = [0]
	VEL = [[0,0,0]]
	MainCounter = 0
	AutoWpNo = 0
	WpNo = 1
	InitFlag = 1
	TiOutFlag = 0
	ImProFail = 0
	t = 0
	AutoWpTh = AutoWpTh - 10
	TimeUpd = np.zeros(len(VisualWps)+1,dtype = 'float')
	WpCounter = len(VisualWps)+2
	MaintStart = time.time()
	Vb = np.zeros(3)
	IpCalDrift = np.zeros(2)
	

	# Determine the wind in AUTO or CIRCLE mode before entering FBWB mode
	TVEC,GPSSTAT,EUANGS,ASP,WindM,WindD,MainCounter,MaintStart,TiOutFlag = AutoB4Fbwb.AutoB4Fbwb(ser,TVEC,MaintStart,AutoWps,AutoWpTh,AutoWpNo,MainCounter,fn,fn1,TrialNo,TiOutFlag,TiOutTh)
	if TiOutFlag == 0:
		#GPSSTAT = [[0,0,0,0]]
		#EUANGS = [[0,0,0]]
		#ASP = [10]
		#WindM = 10
		#WindD = 225	

		print 'WindM = ', WindM
		print 'WindD = ', WindD
		# End of wind calculation
		ti = time.time()
		while 1:
			if ser == 0:								# SIMULATION
				header = "DATA"
				ti = time.time()
				x = raw_input('Press ENTER to continue.')
				if not x:
					ti = time.time()
					break
			else:										# REAL EXPERIMENT
				rcv = ser.inWaiting()
				while rcv == 0:
					rcv = ser.inWaiting()
					DelT = time.time() - ti
					if DelT > 0:
						TiOutFlag = 1
						break
				if TiOutFlag == 1:
					break						
				rcv0 = float(rcv)
				print 'rcv0 b4 while =',rcv0
				print 'Got into FBWB mode'
				while rcv0 != 70:			# Look for first header
					rcv = ser.readline()
					rcv0 = float(rcv)
					print 'rcv0 inside while =',rcv0		
				print 'rcv0 aft while =',rcv0	
				print 'Received 70'	
				rcv = ser.readline()	#	Read second header
				rcv1 = float(rcv)
				if rcv1 == 66:				# Second header received
					SensedRoll = ser.readline()
					SensedPitch = ser.readline()
					SensedYaw = ser.readline()
					SensedAsp = ser.readline()
					SensedCurTime = ser.readline()
					SensedLat = ser.readline()
					SensedLng = ser.readline()
					SensedAlt = ser.readline()
					SensedTail = ser.readline()
					ser.flushInput()
		
					Roll = float(SensedRoll)
					Pitch = float(SensedPitch)
					Yaw = float(SensedYaw)
					Asp = float(SensedAsp)
					if HiL == True:
						Asp,CAng = BasicFns.AspCAngCalc(Asp,Yaw,WindM,WindD)
					CurTime = long(SensedCurTime)
					Lat = float(SensedLat)
					Lng = float(SensedLng)
					Alt = float(SensedAlt)		
					Tail = int(SensedTail)	# Receive Handshaking Signal
					print 'Tail =',Tail
					print 'Initial Yaw =',Yaw
			
					Yaw = Yaw + 100*HeadErr[round(Yaw/10)]
					if Yaw > 36000:
						Yaw = Yaw - 36000
					elif Yaw < 0:
						Yaw = Yaw + 36000
		
					print 'Initial Roll =',Roll
					print 'Initial Pitch =',Pitch
					print 'Initial Yaw =',Yaw
					print 'Initial Air Speed =',Asp
					print 'Current Time =',CurTime
					print 'Latitude =',Lat
					print 'Longitude =',Lng
					print 'Initial Air Speed =',Asp

					hsSignal = "YS"
					ToSend = [hsSignal[0],hsSignal[1]]
					ToSendStr = ''.join(ToSend)
					time.sleep(0.1)
					ser.flushOutput()
					ser.write(ToSendStr)	
					time.sleep(0.1)
				else:										# Both headers were not received (Need to request to send data again)
					hsSignal = "NO"
					ToSend = [hsSignal[0],hsSignal[1]]
					ToSendStr = ''.join(ToSend)
					ser.flushOutput()
					ser.write(ToSendStr)	
					ser.flushInput()

				YorN = ser.inWaiting()
				while YorN == 0:
					YorN = ser.inWaiting()
				while ((YorN != 'y') and (YorN != 'n')):
					YorN = ser.read()
				if YorN == 'y':
					fn.write('########## FBWB MODE ##########' + '\n')
					break
		if TiOutFlag == 0:
			if ser == 0:		
				while WpCounter > 1:
					# Navigation in straight line.
					RollCd = 0
					fn.write('########## FBWB MODE B4 MOVE IN STRAIGHT LINE ##########' + '\n')
					i = len(TimeUpd)-WpCounter + 1
					TimeUpd[i] = 2
					ReqYaw = 0
					RollCd = 0
					RollConst = 0
					X2 = 0
					Y2 = 0
					MainCounter = MainCounter + 1
					t = time.time() - MaintStart
					pos_x = POS[len(POS)-1,0]
					pos_y = POS[len(POS)-1,1]
					pos_z = POS[len(POS)-1,2]
					Roll = 0
					Pitch = 0
					Yaw = 0
					Lat = 0
					Lng = 0
					Alt = 400
					Asp = 0
					alldata = [MainCounter,t,pos_x,pos_y,pos_z,Roll,Pitch,Yaw,Lat,Lng,Alt,Asp]
					fn.write(' '.join(map(str,alldata)) + '\n')	
					TVEC,GPSSTAT,POS,EUANGS,VEL,MainCounter,CAng,ti,ReqYaw,RollCd,RollConst,TiOutFlag = NavFns.SerialOpsMove(ser,header,GPSSTAT,POS,EUANGS,VEL,TVEC,HiL,MaintStart,MainCounter,i,ti,HeadErr,fn,fn1,TimeUpd[i],ReqYaw,RollCd,RollConst,InitFlag,WindM,WindD,X2,Y2,TiOutFlag,TiOutTh)
					if TiOutFlag == 1:
						break
					RollConst = int(RollConst)
					print 'RollConst = ',RollConst
					# End of Navigation in straight line.
	
					if WpCounter > 2:
						print ''
						print 'ATTENTION:: just b4 entering the parallel threads#####'
						print ''
						fn.write('###### FBWB MODE B4 MOVE IN PARALLEL THREADS ######' + '\n')
						Yaw = EUANGS[len(EUANGS)-1,2]/100
						time_b4_impro = time.time()
						# Image Processing in parallel with SeraialOpsMove.
						ParThCounter = 0
						a_stop_event = threading.Event()
						a_ImCapEvent = threading.Event()
						a_tiout_event = threading.Event()
						a_impro_event = threading.Event()
						I = ImagProc()
						T = TravelDurIP()
						It = threading.Thread(target=I.IP,args=(Data1,Data2,Data3,Yaw,WpNo,IpCalDrift,ImProFail,awb,ex,photo_width,photo_height,fn,a_stop_event,a_ImCapEvent,a_tiout_event,a_impro_event))
						Tt = threading.Thread(target=T.TDIP,args=(GPSSTAT,POS,EUANGS,VEL,TVEC,HiL,MaintStart,MainCounter,CAng,WpCounter,ti,HeadErr,fn,fn1,ReqYaw,RollCd,WindM,WindD,ParThCounter,TiOutFlag,TiOutTh,X2,Y2,a_stop_event,a_ImCapEvent,a_tiout_event,a_impro_event))
						It.start()
						Tt.start()
						It.join()
						Tt.join()
						print 'out of thread'
	
						IpCalDrift = q_IpCalDrift.get()
						ImProFail = q_ImProFail.get()
						print 'ImProFail =',ImProFail
						if ImProFail == 1:
							TiOutFlag = 1
						POS = q_POS.get()
						EUANGS = q_EUANGS.get()
						VEL = q_VEL.get()
						GPSSTAT = q_GPSSTAT.get()
						ti = q_ti.get()
						print 'ti =',ti
						MainCounter = q_MainCounter.get()
						print 'MainCounter =',MainCounter
						POSatVisWp = q_POSatVisWp.get()
						print 'POSatVisWp =',POSatVisWp
						CAng = q_CAng.get()
						print 'CAng =',CAng
						TiOutFlag = q_TiOutFlag.get()
						print 'TiOutFlag =',TiOutFlag
						if ImProFail == 1:
							TiOutFlag = 1					
						if TiOutFlag == 1:
							print 'Time Out Inside Thread'
							break
	
						DispDurImPro = [(POS[len(POS)-1,0] - POSatVisWp[0]), (POS[len(POS)-1,1] - POSatVisWp[1])]
						fn1.write('#### Displacement During Image Processing ####' + '\n')	
						fn1.write(' '.join(map(str,DispDurImPro)) + '\n\n')
	
						print 'WpNo = ',WpNo
						print 'IpCalDrift = ',IpCalDrift
	
						WpNo = WpNo + 1
						fn1.write('#### Drift Calculated by Image Processing (IpCalDrift) ####' + '\n')	
						fn1.write(' '.join(map(str,IpCalDrift)) + '\n\n')		

						# Computation of ReqYaw.
						Ipos = np.zeros(3)
						Ipos[0] = POS[len(POS)-1,0]
						Ipos[1] = POS[len(POS)-1,1]
						Ipos[2] = POS[len(POS)-1,2]
						Yaw = EUANGS[len(EUANGS)-1,2]
						if ImgPro == True: # Update Ipos[0] and Ipos[1] if Image processing is used.
							Ipos[0] = POS[len(POS)-1,0] - IpCalDrift[0]
							Ipos[1] = POS[len(POS)-1,1] - IpCalDrift[1]
							fn1.write('#### CAng for correction = ' + str(CAng) + '####' + '\n')
							fn1.write('#### Yaw at correction = ' + str(Yaw) + '####' + '\n')
	#						Ipos[0] = WpC[len(WpC)-WpCounter+1,0] + DispDurImPro[0] + IpCalDrift[0] #+ 32*math.cos((CAng+3000)*cd2r)
	#						Ipos[1] = WpC[len(WpC)-WpCounter+1,1] + DispDurImPro[1] + IpCalDrift[1] #+ 32*math.sin((CAng+3000)*cd2r)		
							POS[len(POS)-1,0] = Ipos[0]
							POS[len(POS)-1,1] = Ipos[1]
			
						X1 = Ipos[0]
						Y1 = Ipos[1]
		
						fn.write('########## Just after image processing update ##########' + '\n')
				WpCounter = WpCounter - 1		
				print 'WpCounter =',WpCounter
				InitFlag = 0

			else:		# REAL Experiment				
				WpC = [[0,0,400]]
				for i in range (0,len(VisualWps)):
					arc = BasicFns.DistanceOnUnitSphere(Lat/1e7,Lng/1e7,VisualWps[i][0]/1e7,VisualWps[i][1]/1e7)
					D2VisWp = arc*6373000		
					ReqAng = BasicFns.HeadAngFromLatLon(Lat/1e7,Lng/1e7,VisualWps[i][0]/1e7,VisualWps[i][1]/1e7)
					if i == 0:
						ReqCourse = ReqAng
						D21stWp = D2VisWp
					XyPlaneAng = 9000 - ReqAng
					if XyPlaneAng < 0:
						XyPlaneAng = XyPlaneAng + 36000
					WpC_x = D2VisWp*math.cos(XyPlaneAng*cd2r)
					WpC_y = D2VisWp*math.sin(XyPlaneAng*cd2r)

					WpC = np.append(WpC,[[WpC_x,WpC_y,400]],axis = 0)
				WpC = np.append(WpC,[[0,0,400]],axis = 0)	
				fn1.write('#### XY Waypoint Coordinates: Initial Position is set when switching was made from Auto to FBWB mode ####' + '\n')	
				fn1.write(' '.join(map(str,WpC)) + '\n')

				MainCounter = MainCounter + 1				
				GPSSTAT = np.append(GPSSTAT,[[CurTime,Lat,Lng,Alt]],axis=0)
				EUANGS = np.append(EUANGS,[[Roll,Pitch,Yaw]],axis=0)

				t = time.time() - MaintStart
				alldata = [MainCounter,t,0,0,0,Roll,Pitch,Yaw,Lat,Lng,Alt,Asp]
				fn.write(' '.join(map(str,alldata)) + '\n')

				Vb[0] = Asp*math.cos(Yaw*cd2r)*math.cos(Pitch*cd2r)
				Vb[1] = Asp*math.sin(Yaw*cd2r)*math.cos(Pitch*cd2r)
				Vb[2] = Asp*math.sin(Pitch*cd2r)
				VEL = [Vb]

				CVel,ReqYaw = BasicFns.YawCvelCalc(Asp,ReqCourse,WindM,WindD)
				print 'ReqYaw after Course and Wind vectors =',ReqYaw
				print 'Course Velocity after Course and Wind Vectors =',CVel
				TimeUpd[0] = D21stWp/Asp
				TimeUpd[0] = TimeUpd[0] - 4.5		# 4.5 seconds for image processing preparation of RPi
				print 'TimeUpd =',TimeUpd

				#***************************************#
				#### InitialTurn Goes Here (If used) ####
				#***************************************#
				X2 = WpC[1,0]
				Y2 = WpC[1,1]
				header = "DATA"
				RollCd = 0
				RollConst = 0
				ti = time.time()
				while WpCounter > 1:
					# Navigation in straight line.
					RollCd = 0
					fn.write('########## FBWB MODE B4 MOVE IN STRAIGHT LINE ##########' + '\n')
					i = len(TimeUpd)-WpCounter + 1
					TVEC,GPSSTAT,POS,EUANGS,VEL,MainCounter,CAng,ti,ReqYaw,RollCd,RollConst,TiOutFlag = NavFns.SerialOpsMove(ser,header,GPSSTAT,POS,EUANGS,VEL,TVEC,HiL,MaintStart,MainCounter,i,ti,HeadErr,fn,fn1,TimeUpd[i],ReqYaw,RollCd,RollConst,InitFlag,WindM,WindD,X2,Y2,TiOutFlag,TiOutTh)
					if TiOutFlag == 1:
						break
					RollConst = int(RollConst)
					print 'RollConst = ',RollConst
					# End of Navigation in straight line.

					if WpCounter > 2:
						print ''
						print 'ATTENTION:: just b4 entering the parallel threads########################################################################'
						print ''
						fn.write('########## FBWB MODE B4 MOVE IN PARALLEL THREADS ##########' + '\n')
						Yaw = EUANGS[len(EUANGS)-1,2]/100
						time_b4_impro = time.time()
						# Image Processing in parallel with SeraialOpsMove.
						ParThCounter = 0
						a_stop_event = threading.Event()
						a_ImCapEvent = threading.Event()
						a_tiout_event = threading.Event()
						a_impro_event = threading.Event()
						I = ImagProc()
						T = TravelDurIP()
						It = threading.Thread(target=I.IP,args=(Data1,Data2,Data3,Yaw,WpNo,IpCalDrift,ImProFail,awb,ex,photo_width,photo_height,fn,a_stop_event,a_ImCapEvent,a_tiout_event,a_impro_event))
						Tt = threading.Thread(target=T.TDIP,args=(GPSSTAT,POS,EUANGS,VEL,TVEC,HiL,MaintStart,MainCounter,CAng,WpCounter,ti,HeadErr,fn,fn1,ReqYaw,RollCd,WindM,WindD,ParThCounter,TiOutFlag,TiOutTh,X2,Y2,a_stop_event,a_ImCapEvent,a_tiout_event,a_impro_event))
						It.start()
						Tt.start()
						It.join()
						Tt.join()
						print 'out of thread'

						IpCalDrift = q_IpCalDrift.get()
						print 'IpCalDrift =',IpCalDrift
						ImProFail = q_ImProFail.get()
						print 'ImProFail =',ImProFail
						if ImProFail == 1:
							TiOutFlag = 1
						POS = q_POS.get()
						EUANGS = q_EUANGS.get()
						VEL = q_VEL.get()
						GPSSTAT = q_GPSSTAT.get()
						ti = q_ti.get()
						print 'ti =',ti
						MainCounter = q_MainCounter.get()
						print 'MainCounter =',MainCounter
						POSatVisWp = q_POSatVisWp.get()
						print 'POSatVisWp =',POSatVisWp
						CAng = q_CAng.get()
						print 'CAng =',CAng
						TiOutFlag = q_TiOutFlag.get()
						print 'TiOutFlag =',TiOutFlag
						if ImProFail == 1:
							TiOutFlag = 1					
						if TiOutFlag == 1:
							print 'Time Out Inside Thread'
							break

						DispDurImPro = [(POS[len(POS)-1,0] - POSatVisWp[0]), (POS[len(POS)-1,1] - POSatVisWp[1])]
						fn1.write('#### Displacement During Image Processing ####' + '\n')	
						fn1.write(' '.join(map(str,DispDurImPro)) + '\n\n')

						print 'WpNo = ',WpNo
						print 'IpCalDrift = ',IpCalDrift

						WpNo = WpNo + 1
						fn1.write('#### Drift Calculated by Image Processing (IpCalDrift) ####' + '\n')	
						fn1.write(' '.join(map(str,IpCalDrift)) + '\n\n')		

						# Computation of ReqYaw.
						Ipos = np.zeros(3)
						Ipos[0] = POS[len(POS)-1,0]
						Ipos[1] = POS[len(POS)-1,1]
						Ipos[2] = POS[len(POS)-1,2]
						Yaw = EUANGS[len(EUANGS)-1,2]
						if ImgPro == True: # Update Ipos[0] and Ipos[1] if Image processing is used.
							Ipos[0] = POS[len(POS)-1,0] - IpCalDrift[0]
							Ipos[1] = POS[len(POS)-1,1] - IpCalDrift[1]
							fn1.write('#### CAng for correction = ' + str(CAng) + '####' + '\n')
							fn1.write('#### Yaw at correction = ' + str(Yaw) + '####' + '\n')
	#						Ipos[0] = WpC[len(WpC)-WpCounter+1,0] + DispDurImPro[0] + IpCalDrift[0] #+ 32*math.cos((CAng+3000)*cd2r)
	#						Ipos[1] = WpC[len(WpC)-WpCounter+1,1] + DispDurImPro[1] + IpCalDrift[1] #+ 32*math.sin((CAng+3000)*cd2r)		
							POS[len(POS)-1,0] = Ipos[0]
							POS[len(POS)-1,1] = Ipos[1]
		
						X1 = Ipos[0]
						Y1 = Ipos[1]
	
						fn.write('########## Just after image processing update ##########' + '\n')

						X2 = WpC[len(WpC)-WpCounter+2][0]
						Y2 = WpC[len(WpC)-WpCounter+2][1]

						Yaw = EUANGS[len(EUANGS)-1,2]

						print 'X1,Y1 =',X1,Y1
						print 'X2,Y2 =',X2,Y2
	
						if X1 == X2: # ReqCourse Calculations
							if Y2 > Y1:
								ReqCourse = 9000
							else:
								ReqCourse = 27000
						else: 
							ReqCourse = math.atan((Y2-Y1)/(X2-X1))
							ReqCourse = ReqCourse*r2cd
		
						if X2 < X1:
							ReqCourse = ReqCourse + 18000
						else:
							if Y2 < Y1:
								ReqCourse = ReqCourse + 36000		
			
						ReqCourse = 9000 - ReqCourse
						if ReqCourse < 0:
							ReqCourse = ReqCourse + 36000		
						# End of ReqCourse Calculations
						CVel,ReqYaw = BasicFns.YawCvelCalc(Asp,ReqCourse,WindM,WindD)
						print 'ReqYaw =',ReqYaw			

						tStart = time.time()
						print 'Yaw for TurnDir calc = ',Yaw
						if abs(ReqYaw -Yaw) < 10:		# Calculation of TurnDir 
							TurnDir = 2
						else:
							if Yaw > ReqYaw:	
								if Yaw-ReqYaw > 18000:
									TurnDir = 1
								else:
									TurnDir = 0
							else:		# condition: ReqYaw > Yaw
								if ReqYaw-Yaw > 18000:
									TurnDir = 0
								else:
									TurnDir = 1		
						# End of Computation of RollCd and ReqYaw		

						# Navigation during turn.
						print '#############################################ATTENTION:: just b4 turn'
						print 'ReqYaw =',ReqYaw
						print 'TurnDir =',TurnDir
						CdToTurn1 = ReqYaw - Yaw
						CdToTurn = abs(CdToTurn1)
						if TurnDir == 0 and CdToTurn1 > 0:
							CdToTurn = 36000 - CdToTurn
						elif TurnDir == 1 and CdToTurn1 < 0:
							CdToTurn = 36000 - CdToTurn
						print 'CdToTurn =',CdToTurn	
						TurnCounter = 0
						YawIncDec = 0
						while TurnCounter < 4:		
							Yaw = EUANGS[len(EUANGS)-1,2]
							if TurnDir == 1:
								ReqYawUpd = ReqYaw - YawIncDec
								if ReqYawUpd < 0:
									ReqYawUpd = ReqYawUpd + 36000
							elif TurnDir == 0:
								ReqYawUpd = ReqYaw + YawIncDec
								if ReqYawUpd > 36000:
									ReqYawUpd = ReqYawUpd - 36000
							if TurnDir == 1:
								if ((ReqYawUpd - Yaw) > 0 or (ReqYawUpd - Yaw) < -18000):
									TVEC,GPSSTAT,POS,EUANGS,VEL,MainCounter,ti,RollCd,YawIncDec,Asp,TiOutFlag = NavFns.SerialOpsTurn(ser,header,GPSSTAT,POS,EUANGS,VEL,TVEC,HiL,MaintStart,MainCounter,ti,HeadErr,Yaw,ReqYaw,YawIncDec,RollCd,RollCdMax,RollConst,TurnDir,CdToTurn,WindM,WindD,TurnCounter,fn,TiOutFlag,TiOutTh)
							elif TurnDir == 0:
								if ((ReqYawUpd - Yaw) < 0 or (ReqYawUpd - Yaw) > 18000):
									TVEC,GPSSTAT,POS,EUANGS,VEL,MainCounter,ti,RollCd,YawIncDec,Asp,TiOutFlag = NavFns.SerialOpsTurn(ser,header,GPSSTAT,POS,EUANGS,VEL,TVEC,HiL,MaintStart,MainCounter,ti,HeadErr,Yaw,ReqYaw,YawIncDec,RollCd,RollCdMax,RollConst,TurnDir,CdToTurn,WindM,WindD,TurnCounter,fn,TiOutFlag,TiOutTh)
							print 'TiOutFlag after Turn = ', TiOutFlag
							if TiOutFlag == 1:
								print 'broken first while loop after turn'
								break		

							TurnCounter = TurnCounter + 1
		
							X1 = POS[len(POS)-1,0]
							Y1 = POS[len(POS)-1,1]	
							print 'Current Position = ',[X1,Y1]
		
							if X1 == X2: # ReqCourse Calculations
								if Y2 > Y1:
									ReqCourse = 9000
								else:
									ReqCourse = 27000
							else: 
								ReqCourse = math.atan((Y2-Y1)/(X2-X1))*r2cd
							print 'Airspeed for TimeUpd calc =',Asp
							if X2 < X1:
								ReqCourse = ReqCourse + 18000
							else:
								if Y2 < Y1:
									ReqCourse = ReqCourse + 36000		

							ReqCourse = 9000 - ReqCourse		# Convert ReqCourse to XY-plane angle
							if ReqCourse < 0:
								ReqCourse = ReqCourse + 36000			
							# End of ReqYaw Calculations
							CVel,ReqYaw = BasicFns.YawCvelCalc(Asp,ReqCourse,WindM,WindD)
							print 'ReqYaw for TurnCounter',TurnCounter,'=',ReqYaw
							# End of ReqYaw Calculations	
						# End of Navigation during turn.
						if TiOutFlag == 1:
							print 'broken second while loop after turn'
							break
					# Calculation of TimeUpd for next section of travel.
					WpCounter = WpCounter - 1		
					print 'WpCounter =',WpCounter

					if WpCounter > 1:
						X1 = POS[len(POS)-1,0]
						Y1 = POS[len(POS)-1,1]

						X2 = WpC[len(WpC)-WpCounter+1][0]
						Y2 = WpC[len(WpC)-WpCounter+1][1]

						print 'WpCounter =',WpCounter
						print 'Current Position =',X1,Y1
						print 'Next Waypoint =',X2,Y2			
						TimeUpd[len(TimeUpd)-WpCounter+1] =  (math.sqrt(pow((X2 - X1),2) + pow((Y2 - Y1),2))/CVel) 
				#		TimeUpd[len(TimeUpd)-WpCounter+1] =  (math.sqrt(pow((X2 - X1),2) + pow((Y2 - Y1),2))/CVel) - 10
						fn1.write('#### TimeUpd for next WP:' + str(TimeUpd[len(TimeUpd)-WpCounter+1]) + '####' + '\n\n\n')
					print 'Airspeed for TimeUpd calc =',Asp
					print 'TimeUpd[len(TimeUpd)-WpCounter] =', TimeUpd[len(TimeUpd)-WpCounter]
					print 'TimeUpd =',TimeUpd	
					# End of calculation of TimeUpd for next section of travel.
					InitFlag = 0
		
	fn.close()
	fn1.close()

	if ser == 0:
		print 'aakash'
		filename = 'FlightData/DataFiles/TrialNo' + str(TrialNo) + '/MainDataFile.txt'
		shutil.copy2('FlightData/MainDataFile.txt', filename)
		filename = 'FlightData/DataFiles/TrialNo' + str(TrialNo) + '/OtherDataFile.txt'
		shutil.copy2('FlightData/OtherDataFile.txt', filename)	
	else:	
		filename = '/home/pi/FlightTest/FlightData/DataFiles/TrialNo' + str(TrialNo) + '/MainDataFile.txt'
		shutil.copy2('/home/pi/FlightTest/FlightData/MainDataFile.txt', filename)
		filename = '/home/pi/FlightTest/FlightData/DataFiles/TrialNo' + str(TrialNo) + '/OtherDataFile.txt'
		shutil.copy2('/home/pi/FlightTest/FlightData/OtherDataFile.txt', filename)

		npPath = '/home/pi/FlightTest/FlightData/FBWB/TrialNo' + str(TrialNo) + '/'
		filename = npPath + 'GPSSTAT.txt'
		np.savetxt(filename,GPSSTAT)
		filename = npPath + 'POS.txt'
		np.savetxt(filename,POS)
		filename = npPath + 'EUANGS.txt'
		np.savetxt(filename,EUANGS)
		filename = npPath + 'VEL.txt'
		np.savetxt(filename,VEL)		
		filename = npPath + 'TVEC.txt'
		np.savetxt(filename,TVEC)	
	
	print 'Navigation: successfully saved data files'
	TrialNo = TrialNo + 1

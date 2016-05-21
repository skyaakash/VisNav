import os
import subprocess
import time
import serial
import numpy as np
import math
import shutil
import BasicFns
import threading
import Queue

q_TVEC = Queue.Queue() 
q_GPSSTAT = Queue.Queue()
q_EUANGS = Queue.Queue()
q_ASP = Queue.Queue()
q_WindM = Queue.Queue()
q_WindD = Queue.Queue()
q_MainCounter = Queue.Queue()
q_MaintStart = Queue.Queue()
q_TiOutFlag = Queue.Queue()

# Classes for Parallel Processing
class ImagCap:
	def IC(self,ser,TrialNo,ImCapEvent,DoneEvent,CapturingImEvent):
		ex = 'auto' 		#'1e-8'
		awb = 'auto' 		#'sun'
		photo_width  = 800
		photo_height = 600	
		WpNo = 0
		while not DoneEvent.is_set():
			cnt = 0
			if ImCapEvent.is_set():
				while cnt < 3:
					CapturingImEvent.set()
					if ser != 0:
						filename = '/home/pi/FlightTest/FlightData/Survey/TrialNo' + str(TrialNo) + '/Wp_' + str(WpNo) + '_' + str(cnt) + '_Image' + awb + '.jpg'
						cmd = 'raspistill -o ' + filename + ' -t 1000 -ss ' + str(ex) + ' -awb ' + awb + ' -w ' + str(photo_width) + ' -h ' + str(photo_height)
						pid = subprocess.call(cmd, shell=True)	
					cnt = cnt + 1
					if cnt == 3:
						ImCapEvent.clear()			
				WpNo = WpNo + 1
				if WpNo == 3:
					DoneEvent.set()		
			else:
				time.sleep(0.1)					
class TravelDurIC: 
	def TDIC(self,ser,TVEC,MaintStart,AutoWps,AutoWpTh,WpNo,MainCounter,fn,fn1,TrialNo,TiOutFlag,TiOutTh,ImCapEvent,DoneEvent,CapturingImEvent):
		if ser != 0:
			ser.flushInput()
		cnt = 10	
		DataAtWpHit = np.zeros(4)
		CurWp = np.zeros(2)
		count = 0	
		tStartFlag = 1
		WindM = 0
		WindD = 0
		d2wp = 50
		while not DoneEvent.is_set():
			ti = time.time()
			if ser == 0:						# Simulation (No Serial Port)
				Roll = 0
				Pitch = 0
				Yaw = 0
				Asp = 20
				CurTime = time.time()
				Lat = 0
				Lng = 0
				Alt = 400
				WindMag = 1
				WindDir = 0
				d2wp = d2wp - Asp*0.1
				time.sleep(0.1)
			else:
				rcv = ser.inWaiting()
				while rcv == 0:
					rcv = ser.inWaiting()
					DelT = time.time() - ti
					if DelT > TiOutTh and tStartFlag == 0:
						TiOutFlag = 1
						break
				if TiOutFlag == 1:
					break					
				if tStartFlag == 1:	
					MaintStart = time.time()	
					tStartFlag = 0
					fn.write('########## AUTO MODE ##########' + '\n')
				SensedRoll = ser.readline()
				SensedPitch = ser.readline()
				SensedYaw = ser.readline()
				SensedAsp = ser.readline()
				SensedCurTime = ser.readline()
				SensedLat = ser.readline()
				SensedLng = ser.readline()
				SensedAlt = ser.readline()
				SensedWm = ser.readline()
				SensedWd = ser.readline()

				Roll = int(SensedRoll)
				Pitch = int(SensedPitch)
				Yaw = int(SensedYaw)
				Asp = float(SensedAsp)
				CurTime = long(SensedCurTime)
				Lat = float(SensedLat)
				Lng = float(SensedLng)
				Alt = float(SensedAlt)
				WindMag = float(SensedWm)
				WindDir = float(SensedWd)
			
				CurWp = AutoWps[WpNo]
				LatDiff = abs(CurWp[0] - Lat) 
				LngDiff = abs(CurWp[1] - Lng)
				LLD = math.sqrt(LatDiff*LatDiff + LngDiff*LngDiff)
				arc = BasicFns.DistanceOnUnitSphere(CurWp[0]/1e7,CurWp[1]/1e7,Lat/1e7,Lng/1e7)
				d2wp = arc*6373000

			if MainCounter == 0:
				GPSSTAT = [[CurTime,Lat,Lng,Alt]]
				EUANGS = [[Roll,Pitch,Yaw]]
				ASP = [Asp]
			else:
				GPSSTAT = np.append(GPSSTAT,[[CurTime,Lat,Lng,Alt]],axis=0)
				EUANGS = np.append(EUANGS,[[Roll,Pitch,Yaw]],axis=0)
				ASP = np.append(ASP,[Asp],axis=0)
			if MainCounter % 10 == 0:
				print 'd2wp =',d2wp 
				print 'Asp =', Asp

			if d2wp < AutoWpTh:
				ImCapEvent.set()
				print '.....................................'
				print 'Hey Aakash Uav Hit the WayPoint: Now it will save data for you. Cheers! Buddy.'
				if WpNo == 0:
					print '1st AUTO Wp'
					fn.write('\n 1st AUTO WP \n')
					tWp1Start = time.time()		
				elif WpNo == 1:
					print '2nd AUTO Wp'
					fn.write('\n 2nd AUTO WP \n')
				elif WpNo == 2:
					print '3rd AUTO Wp'
					fn.write('\n 3rd AUTO WP \n')
					if ser != 0:
						ser.write(chr(0))
				WpNo = WpNo + 1				
								
			MainCounter = MainCounter + 1
			t = time.time() - MaintStart
			TVEC = np.append(TVEC,[t])
			alldata = [MainCounter,t,0,0,0,Roll,Pitch,Yaw,Lat,Lng,Alt,Asp]
			if CapturingImEvent.is_set():
				fn1.write('Data at ImCap \n')
				fn1.write(' '.join(map(str,alldata)) + '\n')
				fn.write('Image Capturing Point \n')
				CapturingImEvent.clear()				
			fn.write(' '.join(map(str,alldata)) + '\n')

		q_TVEC.put(TVEC)
		q_GPSSTAT.put(GPSSTAT)
		q_EUANGS.put(EUANGS)
		q_ASP.put(ASP)
		q_WindM.put(WindM)
		q_WindD.put(WindD)
		q_MainCounter.put(MainCounter)
		q_MaintStart.put(MaintStart)
		q_TiOutFlag.put(TiOutFlag)
								
# End of Classes for Parallel Processing	
			   
def AutoB4Fbwb(ser,TVEC,MaintStart,AutoWps,AutoWpTh,WpNo,MainCounter,fn,fn1,TrialNo,TiOutFlag,TiOutTh):
	a_ImCapEvent = threading.Event()
	a_DoneEvent = threading.Event()
	a_CapturingImEvent = threading.Event()
	I = ImagCap()
	T = TravelDurIC()
	It = threading.Thread(target=I.IC,args=(ser,TrialNo,a_ImCapEvent,a_DoneEvent,a_CapturingImEvent))
	Tt = threading.Thread(target=T.TDIC,args=(ser,TVEC,MaintStart,AutoWps,AutoWpTh,WpNo,MainCounter,fn,fn1,TrialNo,TiOutFlag,TiOutTh,a_ImCapEvent,a_DoneEvent,a_CapturingImEvent))
	It.start()
	Tt.start()
	It.join()
	Tt.join()
	
	TVEC = q_TVEC.get()
	GPSSTAT= q_GPSSTAT.get()
	EUANGS = q_EUANGS.get()
	ASP = q_ASP.get()
	WindM = q_WindM.get()
	WindD = q_WindD.get()
	MainCounter = q_MainCounter.get()
	MaintStart = q_MaintStart.get()
	TiOutFlag = q_TiOutFlag.get()
	if ser == 0:
		filename = 'FlightData/DataFiles/TrialNo' + str(TrialNo) + '/MainDataFile.txt'
		shutil.copy2('FlightData/MainDataFile.txt', filename)
		filename = 'FlightData/DataFiles/TrialNo' + str(TrialNo) + '/OtherDataFile.txt'
		shutil.copy2('FlightData/OtherDataFile.txt', filename)	
	else:	
		filename = '/home/pi/FlightTest/FlightData/DataFiles/TrialNo' + str(TrialNo) + '/MainDataFile.txt'
		shutil.copy2('/home/pi/FlightTest/FlightData/MainDataFile.txt', filename)
		filename = '/home/pi/FlightTest/FlightData/DataFiles/TrialNo' + str(TrialNo) + '/OtherDataFile.txt'
		shutil.copy2('/home/pi/FlightTest/FlightData/OtherDataFile.txt', filename)						
	return TVEC,GPSSTAT,EUANGS,ASP,WindM,WindD,MainCounter,MaintStart,TiOutFlag

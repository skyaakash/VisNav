import numpy as np
import math
import struct
import serial
import time
import BasicFns
cd2r = math.pi/18000
d2r = math.pi/180
r2d = 180/math.pi
r2cd = 18000/math.pi

def Init(WpC,Asp,IpTime):
	l = len(WpC[:,0])
	WpAng = np.zeros(l,dtype = 'float')				# This will set last element of WpAng = 0 (No turn required at the end) (See codes below)
	WpAng1 = np.zeros(l,dtype = 'float')
	# Initialization of other variables	
	Vb = np.zeros(3)

	Ipos = np.zeros(3,dtype ='float')	
	Ipos = WpC[0,:]
	POS = [Ipos]

	# WP to WP angles
	for i in range(0,l-1):
		if i == 0:
			X1 = WpC[i,0]
			Y1 = WpC[i,1]
			Z1 = WpC[i,2]
		else:
			X1 = WpC[i,0]
			Y1 = WpC[i,1]
			Z1 = WpC[i,2]
		X2 = WpC[i+1,0]
		Y2 = WpC[i+1,1]
		Z2 = WpC[i+1,2]
		if X1 == X2:
			if Y2 > Y1:
				WpAng[i] = 90
			else:
				WpAng[i] = 270
		else:
			WpAng[i] = (math.atan(float(Y2-Y1)/(X2-X1)))*r2d
			print 'aakash = ',WpAng[i]
			if X2 > X1:
				if Y2 < Y1:
					WpAng[i] = WpAng[i] + 360		
			else:
					WpAng[i] = WpAng[i] + 180
		XYdist = math.sqrt(pow((X2 - X1),2)	+ pow((Y2 - Y1),2))		
		WpAng1[i] = math.atan((Z2 - Z1)/XYdist)
		WpAng1[i] = WpAng1[i]*r2d
	print 'WpC =',WpC	
	print 'WpAng =',WpAng
	print 'WpAng1 =',WpAng1
	
	TimeUpdDen = math.sqrt(pow((WpC[1,0] - WpC[0,0]),2) + pow((WpC[1,1] - WpC[0,1]),2) + pow((WpC[1,2] - WpC[0,2]),2))
	ReqCourse = WpAng[0]*100
	ReqCourse = 9000 - ReqCourse
	if ReqCourse < 0:
		ReqCourse = ReqCourse + 36000		
	return POS,ReqCourse,TimeUpdDen
# End of Definition Init.		
	
def Move(POS,EUANGS,VEL,Asp,WindM,WindD,DelT):
	Roll = EUANGS[len(EUANGS)-1,0]
	Pitch = EUANGS[len(EUANGS)-1,1]
	Yaw = EUANGS[len(EUANGS)-1,2]
	
	CVel,CAng = BasicFns.SubtractVectors(Asp,Yaw,WindM,WindD)
	
	Ipos = np.zeros(3,dtype = 'float')
	Ipos[0] = POS[len(POS)-1][0]
	Ipos[1] = POS[len(POS)-1][1]
	Ipos[2] = POS[len(POS)-1][2]
	
	Vb = np.zeros(3)
	CAng = 9000 - CAng
	if CAng < 0:
		CAng = 36000 + CAng
	Vb[0],Vb[1],Vb[2] = BasicFns.Euler2Cartesian(CVel,Pitch,CAng)
	Ipos[0] = Ipos[0] + (CVel*DelT)*math.cos(CAng*cd2r)*math.cos(Pitch*cd2r)
	Ipos[1] = Ipos[1] + (CVel*DelT)*math.sin(CAng*cd2r)*math.cos(Pitch*cd2r)
	Ipos[2] = Ipos[2] #+ (VMag*DelT)*math.sin(Pitch*cd2r)		# uncomment this if you require 3D movement (in X,Y,Z-axes)
	
	POS = np.append(POS,[Ipos],axis=0)
	VEL = np.append(VEL,[Vb],axis=0)
	return POS,VEL,CVel,CAng	
# End of Definition Move.		
	
def SerialOpsMove(ser,header,GPSSTAT,POS,EUANGS,VEL,TVEC,HiL,MaintStart,MainCounter,i,ti,HeadErr,fn,fn1,TimeUpd,ReqYaw,RollCd,RollConst,InitFlag,WindM,WindD,X2,Y2,TiOutFlag,TiOutTh):
	print'.............................................................................................................................'
	print 'Moving in St Ln'
	r = 0
	tDiff = 0
	YawConst = EUANGS[len(EUANGS)-1][2]
	YawErrPrev = 0
	UpdFlag = 1
	UpdCounterFlag = 0
	PITCH = [0]
	PitchTimeComp = 0
	tStart = time.time()
	while tDiff < TimeUpd:
		while 1:
			if ser == 0:		# SIMULATION
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

#				MainCounter = MainCounter + 1
#				DelT = time.time() - ti	
#				ti = time.time()
#				POS,VEL,CVel,CAng = Move(POS,EUANGS,VEL,Asp,WindM,WindD,DelT)
#				t = time.time() - MaintStart
#				TVEC = np.append(TVEC,[t])								
#				EUANGS = np.append(EUANGS,[[Roll,Pitch,Yaw]],axis=0)
#				GPSSTAT = np.append(GPSSTAT,[[CurTime,Lat,Lng,Alt]],axis=0)
#				pos_x = POS[len(POS)-1,0]
#				pos_y = POS[len(POS)-1,1]
#				pos_z = POS[len(POS)-1,2]
#				tDiff = time.time() - tStart
				time.sleep(0.1)
				if tDiff > TimeUpd:
					break

			else:		# REAL EXPERIMENT
				h = [elem.encode("hex") for elem in header]
				dahex = struct.pack('f',RollCd).encode('hex')
				d = bytearray.fromhex(dahex)
				chk = int(h[0],16)^int(h[1],16)^int(h[2],16)^int(h[3],16)^int(d[0])^int(d[1])^int(d[2])^int(d[3])		
				ToSend = [header[0],header[1],header[2],header[3],chr(d[0]),chr(d[1]),chr(d[2]),chr(d[3]),chr(chk)]
				ToSendStr = ''.join(ToSend)
				ser.flushInput()
				ser.flushOutput()
				ser.write(ToSendStr)	
				rInt = 0
				inw_data = ser.inWaiting()
				while inw_data == 0:
					inw_data = ser.inWaiting()
					DelT = time.time() - ti
					if DelT > TiOutTh:
						TiOutFlag = 1
						print 'timeout'
						break
				if TiOutFlag == 1:
					print 'timeout'
					break		
					
				hdr0 = 0	
				while hdr0 != 68:
					hdr = ser.readline()
					hdr0 = int(hdr)
					DelT = time.time() - ti
					if DelT > TiOutTh:
						TiOutFlag = 1
						print 'timeout'
						break				
			
				hdr = ser.readline()
				hdr1 = int(hdr)
				if hdr1 == 65:
					hdr = ser.readline()
					hdr2 = int(hdr)
					if hdr2 == 84:
						hdr = ser.readline()
						hdr3 = int(hdr)
						if hdr3 == 65:	
							r = ser.readline()
							rInt = int(r)
						
				if rInt == 1:
					f = ser.readline()		
					SensedRoll = ser.readline()
					SensedPitch = ser.readline()
					SensedYaw = ser.readline()
					SensedAsp = ser.readline()
					SensedCurTime = ser.readline()
					SensedLat = ser.readline()
					SensedLng = ser.readline()
					SensedAlt = ser.readline()
					SensedTail = ser.readline()

					Roll = int(SensedRoll)
					Pitch = int(SensedPitch)
					Yaw = int(SensedYaw)
					Asp = float(SensedAsp)
					if HiL == True:
						Asp,CAng = BasicFns.AspCAngCalc(Asp,Yaw,WindM,WindD)
					CurTime = long(SensedCurTime)
					Lat = float(SensedLat)
					Lng = float(SensedLng)
					Alt = float(SensedAlt)
					Tail = int(SensedTail)  # Receive Handshaking Signal
				
					PITCH = np.append(PITCH,[Pitch],axis = 0) 
			
					hsSignal = "YS"
					ToSend = [hsSignal[0],hsSignal[1]]
					ToSendStr = ''.join(ToSend)
					ser.flushInput()
					ser.flushOutput()
					ser.write(ToSendStr)	# Send Handshaking Signal
		
					Yaw = Yaw + 100*HeadErr[round(Yaw/10)]
					if Yaw > 36000:
						Yaw = Yaw - 36000
					elif Yaw < 0:
						Yaw = Yaw + 36000		
				else:
					hsSignal = "NO"
					ToSend = [hsSignal[0],hsSignal[1]]
					ToSendStr = ''.join(ToSend)
					ser.flushInput()
					ser.flushOutput()
					ser.write(ToSendStr)	# Send Handshaking Signal									
					print 'Oh No!!!!'
				
			
				YorN = ser.inWaiting()
				while YorN == 0:
					YorN = ser.inWaiting()
					DelT = time.time() - ti
					if DelT > TiOutTh:
						TiOutFlag = 1
						print 'timeout'
						break	
				if TiOutFlag == 1:
					print 'timeout'
					break					
				
				while ((YorN != 'y') and (YorN != 'n')):
					YorN = ser.read()
				if YorN == 'y':
					break	
			if TiOutFlag == 1:
				print 'timeout'
				break													
						
			# This section of code is for cotrolling Yaw using Roll
			YawErr = ReqYaw - Yaw
			YawErrDiff = YawErr - YawErrPrev
			if YawErrDiff > 30000:
				YawErrDiff = YawErrDiff - 36000
			elif YawErrDiff < -30000:
				YawErrDiff = YawErrDiff + 36000	 
			RollCd = RollCd + (YawErrDiff)
			YawErrPrev = YawErr
	
			# Initial Value of Roll: Roll that keeps plane in constant course.
			if InitFlag == 1:
				RollConst = RollCd			

			MainCounter = MainCounter + 1
			DelT = time.time() - ti	
			ti = time.time()
			POS,VEL,CVel,CAng = Move(POS,EUANGS,VEL,Asp,WindM,WindD,DelT)
			t = time.time() - MaintStart
			TVEC = np.append(TVEC,[t])								
			EUANGS = np.append(EUANGS,[[Roll,Pitch,Yaw]],axis=0)
			GPSSTAT = np.append(GPSSTAT,[[CurTime,Lat,Lng,Alt]],axis=0)
			pos_x = POS[len(POS)-1,0]
			pos_y = POS[len(POS)-1,1]
			pos_z = POS[len(POS)-1,2]
			alldata = [MainCounter,t,pos_x,pos_y,pos_z,Roll,Pitch,Yaw,Lat,Lng,Alt,Asp]
			fn.write(' '.join(map(str,alldata)) + '\n')				

			tDiff = time.time() - tStart		
			if ((tDiff > 5) and (TimeUpd > 10)):
				tStart = time.time()
				X1 = POS[len(POS)-1,0]
				Y1 = POS[len(POS)-1,1]
				CurDist = math.sqrt(pow((X2 - X1),2) + pow((Y2 - Y1),2))
				print 'CurDist =',CurDist
				print 'CVel =',CVel
				TimeUpd = (float(CurDist)/CVel)
				TimeUpd = TimeUpd - 4.5	#+ (i+1)*0.8					# 5 seconds for image processing preparation of RPi
				print 'Time Upd =',TimeUpd		
				if CurDist > 120:
					if X1 == X2: 									# ReqCourse Calculations
						if Y2 > Y1:
							ReqCourse = 9000
						else:
							ReqCourse = 27000
					else: 
						ReqCourse = math.atan((Y2-Y1)/(X2-X1))*r2cd
					if X2 < X1:
						ReqCourse = ReqCourse + 18000
					else:
						if Y2 < Y1:
							ReqCourse = ReqCourse + 36000		

					ReqCourse = 9000 - ReqCourse
					if ReqCourse < 0:
						ReqCourse = ReqCourse + 36000				
					CVel,ReqYaw = BasicFns.YawCvelCalc(Asp,ReqCourse,WindM,WindD)		
					print 'ReqYaw for update =',ReqYaw		
					TimeUpd = (float(CurDist)/CVel) 
					TimeUpd = TimeUpd - 4.5  #+ (i+1)*0.8 	# 5 seconds for image processing preparation of RPi
					if len(PITCH) > 100:
						Pitch4WT = sum(PITCH[len(PITCH)-50:len(PITCH)])/50
						WpThComp = (float(Alt)/100)*math.tan(Pitch4WT*cd2r)
						PitchTimeComp = WpThComp/Asp
						print 'PitchTimeComp =',PitchTimeComp
						TimeUpd = TimeUpd - PitchTimeComp
				print 'Time Upd =',TimeUpd		

	if ser != 0:		#show only for real experiments
		print'.............................................................................................................................'				
		print 'Message Header =',hdr0,hdr1,hdr2,hdr3
		print 'receivedFlag (from arduino) =',r		
		print 'float RollCd from arduino =',f
		print 'Roll from Plane =',Roll
		print 'Pitch from Plane =',Pitch
		print 'Yaw from Plane =',Yaw
		print 'Airspeed from Plane =',Asp		
		print 'Current Time =',CurTime
		print 'Latitude =', Lat
		print 'Longitude =',Lng
		print 'Altitude =',Alt
		fn1.write('#### PitchTimeComp =' + str(PitchTimeComp) + ' ####' + '\n')	
		fn1.write('#### Time Update =' + str(TimeUpd) + ' ####' + '\n')	
#	np.savetxt('/home/pi/FlightTest/FlightData/GPSSTAT.txt',GPSSTAT)
#	np.savetxt('/home/pi/FlightTest/FlightData/POS.txt',POS)
#	np.savetxt('/home/pi/FlightTest/FlightData/EUANGS.txt',EUANGS)
#	np.savetxt('/home/pi/FlightTest/FlightData/VEL.txt',VEL)
	return TVEC,GPSSTAT,POS,EUANGS,VEL,MainCounter,CAng,ti,ReqYaw,RollCd,RollConst,TiOutFlag
# End of Definition SerialOpsMove.	

def SerialOpsMoveDurImPro(ser,header,GPSSTAT,POS,EUANGS,VEL,TVEC,HiL,MaintStart,MainCounter,CAng,ti,HeadErr,fn,ReqYaw,tStart,WindM,WindD,YawErrPrev,RollCd,X2,Y2,ParThCounter,TiOutFlag,TiOutTh):
	if ParThCounter % 100 == 0:
		print'.............................................................................................................................'
		print 'Moving in St Ln during Image processing'
		print'.............................................................................................................................'
	ParThCounter = ParThCounter + 1	
	r = 0
	while 1:
		if ser == 0:
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

			MainCounter = MainCounter + 1
			DelT = time.time() - ti	
			ti = time.time()
			POS,VEL,CVel,CAng = Move(POS,EUANGS,VEL,Asp,WindM,WindD,DelT)
			t = time.time() - MaintStart
			TVEC = np.append(TVEC,[t])								
			EUANGS = np.append(EUANGS,[[Roll,Pitch,Yaw]],axis=0)
			GPSSTAT = np.append(GPSSTAT,[[CurTime,Lat,Lng,Alt]],axis=0)
			pos_x = POS[len(POS)-1,0]
			pos_y = POS[len(POS)-1,1]
			pos_z = POS[len(POS)-1,2]
			alldata = [MainCounter,t,pos_x,pos_y,pos_z,Roll,Pitch,Yaw,Lat,Lng,Alt,Asp]
			fn.write(' '.join(map(str,alldata)) + '\n')	
			time.sleep(0.1)
			break
		else:
#			time.sleep(0.05)
			h = [elem.encode("hex") for elem in header]
			dahex = struct.pack('f',RollCd).encode('hex')
			d = bytearray.fromhex(dahex)
			chk = int(h[0],16)^int(h[1],16)^int(h[2],16)^int(h[3],16)^int(d[0])^int(d[1])^int(d[2])^int(d[3])		
			ToSend = [header[0],header[1],header[2],header[3],chr(d[0]),chr(d[1]),chr(d[2]),chr(d[3]),chr(chk)]
			ToSendStr = ''.join(ToSend)
			ser.flushInput()
			ser.flushOutput()
			ser.write(ToSendStr)
			rInt = 0	
			inw_data = ser.inWaiting()
			while inw_data == 0:
				inw_data = ser.inWaiting()
				DelT = time.time() - ti
				if DelT > TiOutTh:
					TiOutFlag = 1
					break
			if TiOutFlag == 1:
				break					
						
			hdr0 = 0	
			while hdr0 != 68:
				hdr = ser.readline()
				hdr0 = int(hdr)
				DelT = time.time() - ti
				if DelT > TiOutTh:
					TiOutFlag = 1
					print 'timeout'
					break			
			
			hdr = ser.readline()
			hdr1 = int(hdr)	
			if hdr1 == 65:
				hdr = ser.readline()
				hdr2 = int(hdr)
				if hdr2 == 84:
					hdr = ser.readline()
					hdr3 = int(hdr)
					if hdr3 == 65:	
						r = ser.readline()
						rInt = int(r)		
						
			if rInt == 1:
				f = ser.readline()		
				SensedRoll = ser.readline()
				SensedPitch = ser.readline()
				SensedYaw = ser.readline()
				SensedAsp = ser.readline()
				SensedCurTime = ser.readline()
				SensedLat = ser.readline()
				SensedLng = ser.readline()
				SensedAlt = ser.readline()
				SensedTail = ser.readline()

				Roll = int(SensedRoll)
				Pitch = int(SensedPitch)
				Yaw = int(SensedYaw)
				Asp = float(SensedAsp)
				if HiL == True:
					Asp,CAng = BasicFns.AspCAngCalc(Asp,Yaw,WindM,WindD)			
				CurTime = long(SensedCurTime)
				Lat = float(SensedLat)
				Lng = float(SensedLng)
				Alt = float(SensedAlt)
				Tail = int(SensedTail)  # Receive Handshaking Signal
			
				hsSignal = "YS"
				ToSend = [hsSignal[0],hsSignal[1]]
				ToSendStr = ''.join(ToSend)
				ser.flushInput()
				ser.flushOutput()
				ser.write(ToSendStr)	# Send Handshaking Signal
		
				Yaw = Yaw + 100*HeadErr[round(Yaw/10)]
				if Yaw > 36000:
					Yaw = Yaw - 36000
				elif Yaw < 0:
					Yaw = Yaw + 36000		
			else:
				hsSignal = "NO"
				ToSend = [hsSignal[0],hsSignal[1]]
				ToSendStr = ''.join(ToSend)
				ser.flushInput()
				ser.flushOutput()
				ser.write(ToSendStr)	# Send Handshaking Signal									
			
			YorN = ser.inWaiting()
			while YorN == 0:
				YorN = ser.inWaiting()
				DelT = time.time() - ti
				if DelT > TiOutTh:
					TiOutFlag = 1
					print 'timeout'
					break	
			if TiOutFlag == 1:
				print 'timeout'
				break						
			while ((YorN != 'y') and (YorN != 'n')):
				YorN = ser.read()
			if YorN == 'y':
				break	
		if TiOutFlag == 0:	
			# This section of code is for cotrolling Yaw using Roll
			YawErr = ReqYaw - Yaw
			YawErrDiff = YawErr - YawErrPrev
			if YawErrDiff > 30000:
				YawErrDiff = YawErrDiff - 36000
			elif YawErrDiff < -30000:
				YawErrDiff = YawErrDiff + 36000	 
			RollCd = RollCd + (YawErrDiff)
			YawErrPrev = YawErr

			MainCounter = MainCounter + 1
			DelT = time.time() - ti
			ti = time.time()
			POS,VEL,CVel,CAng = Move(POS,EUANGS,VEL,Asp,WindM,WindD,DelT)
			t = time.time() - MaintStart
			TVEC = np.append(TVEC,[t])		
			EUANGS = np.append(EUANGS,[[Roll,Pitch,Yaw]],axis=0)
			GPSSTAT = np.append(GPSSTAT,[[CurTime,Lat,Lng,Alt]],axis=0)
			pos_x = POS[len(POS)-1,0]
			pos_y = POS[len(POS)-1,1]
			pos_z = POS[len(POS)-1,2]
			alldata = [MainCounter,t,pos_x,pos_y,pos_z,Roll,Pitch,Yaw,Lat,Lng,Alt,Asp]
			fn.write(' '.join(map(str,alldata)) + '\n')		

			tDiff = time.time() - tStart	
			if tDiff > 5:
				tStart = time.time()
				X1 = POS[len(POS)-1,0]
				Y1 = POS[len(POS)-1,1]
				CurDist = math.sqrt(pow((X2 - X1),2) + pow((Y2 - Y1),2))
				print 'CurDist from last waypoint =',CurDist
				print 'CVel =',CVel				
	return TVEC,GPSSTAT,POS,EUANGS,VEL,CAng,MainCounter,ti,tStart,YawErrPrev,ParThCounter,TiOutFlag
# End of Definition SerialOpsMoveDurImPro.	
	
def SerialOpsTurn(ser,header,GPSSTAT,POS,EUANGS,VEL,TVEC,HiL,MaintStart,MainCounter,ti,HeadErr,Yaw,ReqYaw,YawIncDec,RollCd,RollCdMax,RollConst,TurnDir,CdToTurn,WindM,WindD,TurnCounter,fn,TiOutFlag,TiOutTh):
	r = 0	
	RollFlag = 0
	k = 5
	YawPrev = Yaw
	YawInit = Yaw
	print 'Initial Yaw before turn = ',YawInit
	if TurnDir == 1:
		print 'TurnDir = 1'
		RollCd = (RollCdMax + RollConst)
		while 1:
			while 1:
				h = [elem.encode("hex") for elem in header]
				dahex = struct.pack('f',RollCd).encode('hex')
				d = bytearray.fromhex(dahex)
				chk = int(h[0],16)^int(h[1],16)^int(h[2],16)^int(h[3],16)^int(d[0])^int(d[1])^int(d[2])^int(d[3])		
				ToSend = [header[0],header[1],header[2],header[3],chr(d[0]),chr(d[1]),chr(d[2]),chr(d[3]),chr(chk)]
				ToSendStr = ''.join(ToSend)

				ser.flushInput()
				ser.flushOutput()
				ser.write(ToSendStr)		
				rInt = 0	
				inw_data = ser.inWaiting()
				while inw_data == 0:
					inw_data = ser.inWaiting()		
					DelT = time.time() - ti
					if DelT > TiOutTh:
						TiOutFlag = 1
						break
				if TiOutFlag == 1:
					break												
				hdr0 = 0	
				while hdr0 != 68:
					hdr = ser.readline()
					hdr0 = int(hdr)
					DelT = time.time() - ti
					if DelT > TiOutTh:
						TiOutFlag = 1
						print 'timeout'
						break					
					
				hdr = ser.readline()
				hdr1 = int(hdr)
				
				if hdr1 == 65:
					hdr = ser.readline()
					hdr2 = int(hdr)
					if hdr2 == 84:
						hdr = ser.readline()
						hdr3 = int(hdr)
						if hdr3 == 65:	
							r = ser.readline()
							rInt = int(r)	
								
				if rInt == 1:
					f = ser.readline()		
					SensedRoll = ser.readline()
					SensedPitch = ser.readline()
					SensedYaw = ser.readline()
					SensedAsp = ser.readline()
					SensedCurTime = ser.readline()
					SensedLat = ser.readline()
					SensedLng = ser.readline()
					SensedAlt = ser.readline()
					SensedTail = ser.readline()
		
					Roll = int(SensedRoll)
					Pitch = int(SensedPitch)
					Yaw = int(SensedYaw)
					Asp = float(SensedAsp)
					if HiL == True:
						Asp,CAng = BasicFns.AspCAngCalc(Asp,Yaw,WindM,WindD)					
					CurTime = long(SensedCurTime)
					Lat = float(SensedLat)
					Lng = float(SensedLng)
					Alt = float(SensedAlt)
					Tail = int(SensedTail)  # Receive Handshaking Signal
					
					hsSignal = "YS"
					ToSend = [hsSignal[0],hsSignal[1]]
					ToSendStr = ''.join(ToSend)
					ser.flushInput()
					ser.flushOutput()
					ser.write(ToSendStr)	# Send Handshaking Signal
				
					Yaw = Yaw + 100*HeadErr[round(Yaw/10)]
					if Yaw > 36000:
						Yaw = Yaw - 36000
					elif Yaw < 0:
						Yaw = Yaw + 36000		
				else:
					hsSignal = "NO"
					ToSend = [hsSignal[0],hsSignal[1]]
					ToSendStr = ''.join(ToSend)
					ser.flushInput()
					ser.flushOutput()
					ser.write(ToSendStr)	# Send Handshaking Signal									
					print 'Oh No!!!!'
					
				YorN = ser.inWaiting()
				while YorN == 0:
					YorN = ser.inWaiting()
					DelT = time.time() - ti
					if DelT > TiOutTh:
						TiOutFlag = 1
						print 'timeout'
						break	
				if TiOutFlag == 1:
					print 'timeout'
					break							
				while ((YorN != 'y') and (YorN != 'n')):
					YorN = ser.read()					
				if YorN == 'y':
					break			
			
			if TiOutFlag == 1:
				break
			
			MainCounter = MainCounter + 1
			DelT = time.time() - ti
			ti = time.time()
			POS,VEL,CVel,CAng = Move(POS,EUANGS,VEL,Asp,WindM,WindD,DelT)			
			t = time.time() - MaintStart
			TVEC = np.append(TVEC,[t])				
			GPSSTAT = np.append(GPSSTAT,[[CurTime,Lat,Lng,Alt]],axis=0)
			EUANGS = np.append(EUANGS,[[Roll,Pitch,Yaw]],axis=0)	

			pos_x = POS[len(POS)-1,0]
			pos_y = POS[len(POS)-1,1]
			pos_z = POS[len(POS)-1,2]
			alldata = [MainCounter,t,pos_x,pos_y,pos_z,Roll,Pitch,Yaw,Lat,Lng,Alt,Asp]
			fn.write(' '.join(map(str,alldata)) + '\n')			

			YawInc = Yaw - YawInit
			if YawInc < 0:
				YawInc = YawInc + 36000
			if YawInc > CdToTurn/2 and YawInc < 30000 and RollFlag == 0:
				print 'Turn is broken (Half of Turn Achieved)'
				YawIncInRollInc = YawInc
				k = k * 0.75				
				break	
			if TurnCounter == 0:			
				if RollCd >= RollCdMax + RollConst and RollFlag == 0:
					YawIncInRollInc = Yaw - YawInit
					if YawIncInRollInc < -18000:
						YawIncInRollInc = YawIncInRollInc + 36000
					ReqYaw = ReqYaw - YawIncInRollInc
					if ReqYaw < 0:
						ReqYaw = ReqYaw + 36000
					elif ReqYaw >= 36000:						#added later
						ReqYaw = ReqYaw - 36000	
					print 'YawIncInRollInc =',YawIncInRollInc
					print 'ReqYawUpdate = ',ReqYaw	
					RollFlag = 1		
					YawIncDec = YawIncInRollInc
			else:
				if RollFlag == 0:
					ReqYaw = ReqYaw - YawIncDec
					if ReqYaw < 0:
						ReqYaw = ReqYaw + 36000
					elif ReqYaw >= 36000:						#added later
						ReqYaw = ReqYaw = 36000	
					print 'YawIncDec =',YawIncDec
					print 'ReqYawUpdate = ',ReqYaw					
					RollFlag = 1
			Yaw1 = Yaw
			ReqYaw1 = ReqYaw	
			if (YawPrev - Yaw) > 35000:
				Yaw1 = Yaw + 36000
				ReqYaw1 = ReqYaw + 36000
			if (YawPrev < ReqYaw) and (Yaw > ReqYaw or Yaw1 > ReqYaw1):
				print 'Turn is broken'
				print 'Yaw at turn break =',Yaw
				print 'YawPrev at turn break =',YawPrev
				break				
			YawPrev = Yaw	
	else:
		print 'TurnDir = 0'	
		RollPrev = 0				
		RollCd = -(RollCdMax + RollConst)
		while 1:
			while 1:
				h = [elem.encode("hex") for elem in header]
				dahex = struct.pack('f',RollCd).encode('hex')
				d = bytearray.fromhex(dahex)
				chk = int(h[0],16)^int(h[1],16)^int(h[2],16)^int(h[3],16)^int(d[0])^int(d[1])^int(d[2])^int(d[3])		
				ToSend = [header[0],header[1],header[2],header[3],chr(d[0]),chr(d[1]),chr(d[2]),chr(d[3]),chr(chk)]
				ToSendStr = ''.join(ToSend)
				ser.flushInput()		
				ser.flushOutput()
				ser.write(ToSendStr)	
				rInt = 0	
				inw_data = ser.inWaiting()
				while inw_data == 0:
					inw_data = ser.inWaiting()	
					DelT = time.time() - ti
					if DelT > TiOutTh:
						TiOutFlag = 1
						break
				if TiOutFlag == 1:
					break													
				hdr0 = 0	
				while hdr0 != 68:
					hdr = ser.readline()
					hdr0 = int(hdr)
					DelT = time.time() - ti
					if DelT > TiOutTh:
						TiOutFlag = 1
						print 'timeout'
						break
				
				hdr = ser.readline()
				hdr1 = int(hdr)
				
				if hdr1 == 65:
					hdr = ser.readline()
					hdr2 = int(hdr)
					if hdr2 == 84:
						hdr = ser.readline()
						hdr3 = int(hdr)
						if hdr3 == 65:	
							r = ser.readline()
							rInt = int(r)										
				if rInt == 1:
					f = ser.readline()		
					SensedRoll = ser.readline()
					SensedPitch = ser.readline()
					SensedYaw = ser.readline()
					SensedAsp = ser.readline()
					SensedCurTime = ser.readline()
					SensedLat = ser.readline()
					SensedLng = ser.readline()
					SensedAlt = ser.readline()
					SensedTail = ser.readline()
		
					Roll = int(SensedRoll)
					Pitch = int(SensedPitch)
					Yaw = int(SensedYaw)
					Asp = float(SensedAsp)
					if HiL == True:
						Asp,CAng = BasicFns.AspCAngCalc(Asp,Yaw,WindM,WindD)					
					CurTime = long(SensedCurTime)
					Lat = float(SensedLat)
					Lng = float(SensedLng)
					Alt = float(SensedAlt)
					Tail = int(SensedTail)  # Receive Handshaking Signal
					
					hsSignal = "YS"
					ToSend = [hsSignal[0],hsSignal[1]]
					ToSendStr = ''.join(ToSend)
					ser.flushInput()
					ser.flushOutput()
					ser.write(ToSendStr)	# Send Handshaking Signal
				
					Yaw = Yaw + 100*HeadErr[round(Yaw/10)]
					if Yaw > 36000:
						Yaw = Yaw - 36000
					elif Yaw < 0:
						Yaw = Yaw + 36000		
				else:
					hsSignal = "NO"
					ToSend = [hsSignal[0],hsSignal[1]]
					ToSendStr = ''.join(ToSend)
					ser.flushInput()
					ser.flushOutput()
					ser.write(ToSendStr)	# Send Handshaking Signal						
					print 'Oh No!!!!'
					
				YorN = ser.inWaiting()
				while YorN == 0:
					YorN = ser.inWaiting()
					DelT = time.time() - ti
					if DelT > TiOutTh:
						TiOutFlag = 1
						print 'timeout'
						break	
				if TiOutFlag == 1:
					print 'timeout'
					break							
				while ((YorN != 'y') and (YorN != 'n')):
					YorN = ser.read()
				if YorN == 'y':
					break			
			
			if TiOutFlag == 1:
				break
			MainCounter = MainCounter + 1						
			DelT = time.time() - ti
			ti = time.time()
			POS,VEL,CVel,CAng = Move(POS,EUANGS,VEL,Asp,WindM,WindD,DelT)				
			t = time.time() - MaintStart
			TVEC = np.append(TVEC,[t])				
			GPSSTAT = np.append(GPSSTAT,[[CurTime,Lat,Lng,Alt]],axis=0)
			EUANGS = np.append(EUANGS,[[Roll,Pitch,Yaw]],axis=0)
		
			pos_x = POS[len(POS)-1,0]
			pos_y = POS[len(POS)-1,1]
			pos_z = POS[len(POS)-1,2]
			alldata = [MainCounter,t,pos_x,pos_y,pos_z,Roll,Pitch,Yaw,Lat,Lng,Alt,Asp]
			fn.write(' '.join(map(str,alldata)) + '\n')					
			
			YawDec = YawInit - Yaw
			if YawDec < 0:
				YawDec = YawDec + 36000
			if YawDec > CdToTurn/2 and YawDec < 30000 and RollFlag == 0:
				print 'Turn is broken (Half of Turn Achieved)'
				YawDecInRollDec = YawDec
				k = k * 0.75
				break	
			if TurnCounter == 0:	
				if RollCd <= -RollCdMax + RollConst and RollFlag == 0:
					YawDecInRollDec = YawInit - Yaw
					if YawDecInRollDec < -18000:
						YawDecInRollDec = YawDecInRollDec + 36000
					ReqYaw = ReqYaw + YawDecInRollDec
					if ReqYaw > 36000:
						ReqYaw = ReqYaw - 36000		
					elif ReqYaw < 0:			#added later
						ReqYaw = ReqYaw + 36000	
					print 'YawDecInRollDec = ',YawDecInRollDec				
					print 'ReqYawUpdate = ',ReqYaw		
					RollFlag = 1
					YawIncDec = YawDecInRollDec
			else:
				if RollFlag == 0:
					ReqYaw = ReqYaw + YawIncDec
					if ReqYaw > 36000:
						ReqYaw = ReqYaw - 36000	
					elif ReqYaw < 0:			#added later
						ReqYaw = ReqYaw + 36000								
					print 'YawIncDec = ',YawIncDec				
					print 'ReqYawUpdate = ',ReqYaw
					RollFlag = 1	
			ReqYaw1 = ReqYaw			
			if (Yaw - YawPrev) > 35000:
				YawPrev = YawPrev + 36000
				if ReqYaw < 18000:
					ReqYaw1 = ReqYaw + 36000
			if (YawPrev > ReqYaw and Yaw < ReqYaw) or (YawPrev > ReqYaw1 and Yaw < ReqYaw1):
				print 'Turn is broken'
				print 'Yaw at turn break =',Yaw
				print 'YawPrev at turn break =',YawPrev
				break
			YawPrev = Yaw
	print'.............................................................................................................................'							
	print 'Message Header =',hdr0,hdr1,hdr2,hdr3					
	print 'receivedFlag (from arduino) =',r		
	print 'float RollCd from arduino =',f
	print 'Roll from Plane =',Roll
	print 'Pitch from Plane =',Pitch
	print 'Yaw from Plane =',Yaw
	print 'Airspeed from Plane =',Asp		
	print 'TiOutFlag inside Turn Function = ', TiOutFlag
	RollCd = RollConst
	return TVEC,GPSSTAT,POS,EUANGS,VEL,MainCounter,ti,RollCd,YawIncDec,Asp,TiOutFlag
# End of Definition SerialOpsTurn.		

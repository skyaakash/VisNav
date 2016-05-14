import cv2
import numpy as np
import matplotlib.pyplot as plt
import BasicFns
from operator import sub
from itertools import imap
Data = 0
curWpNo = 0
GenData = False
Thresholds = np.load('Database/Wp1Thresholds.npy')
def LoadData(curWpNo):
	global Data,Thresholds
	print 'aakash'
	Data = np.load('Database/Wp'+str(curWpNo)+'Data.npy')
#	Thresholds = np.load('Database/Wp'+str(curWpNo)+'Thresholds.npy')
#	print Thresholds

# AH includes
import math
import pylab
import matplotlib.cm as cm
import time

# FeatSig includes
from scipy.misc import imresize
from scipy.misc import imrotate

# LdmrkDet includes and thresholds
import itertools
NLdmrk = Thresholds[0][0]
ImageTh = Thresholds[1]
BinaryTh = Thresholds[2]
print 'BinaryTh = ',BinaryTh
BinThIncDec = Thresholds[3]
BwAreaTh = Thresholds[4]
LdmrkAngTh = Thresholds[5]
ImCropTh = Thresholds[6]

#WD Thresholds
WpAngTh = Thresholds[7]
WpSiRatTh = Thresholds[8]

#Variables for Generating Data
LdmrkAngsCal0 = [[0.0 for x in range(NLdmrk-1)] for y in range(NLdmrk)]
LdmrkAngsCal1 = [[0.0 for x in range(NLdmrk-2)] for y in range(NLdmrk-1)]
LdmrkAngs0 = [[0.0 for x in range(NLdmrk-1)] for y in range(NLdmrk)]
LdmrkAngs1 = [[[0.0 for x in range(NLdmrk-2)] for y in range(NLdmrk-1)] for z in range(NLdmrk)]
LdmrkAngsTemp = [[0.0 for x in range(NLdmrk-2)] for y in range(NLdmrk-1)]
SoS0 = 0.0
SoS1 = [0.0 for x in range(NLdmrk)]
SoSTemp = 0.0
SUM0 = [0 for x in range(ImCropTh[0])]
SUM1 = [[0 for x in range(ImCropTh[0])] for y in range(NLdmrk)]
SUMTemp = [0 for x in range(ImCropTh[0])]
WpAngs0 = [[0.0 for x in range(NLdmrk-1)] for y in range(NLdmrk)]
WpAngs1 = [[[0.0 for x in range(NLdmrk-2)] for y in range(NLdmrk-1)] for z in range(NLdmrk)]
WpAngsTemp = [[0.0 for x in range(NLdmrk-2)] for y in range(NLdmrk-1)]
WpSideRat0 = [[0.0 for x in range(NLdmrk-1)] for y in range(NLdmrk)]
WpSideRat1 = [[[0.0 for x in range(NLdmrk-2)] for y in range(NLdmrk-1)] for z in range(NLdmrk)]
WpSideRatTemp = [[0.0 for x in range(NLdmrk-2)] for y in range(NLdmrk-1)]
RefPt0 = [0.0, 0.0]
RefPt1 = [[0.0 for x in range(2)] for y in range(NLdmrk)]
RefPtTemp = [0.0 for x in range(2)]
AltRef0 = 1.0
YawRef0 = 0.0


# AngularHistogram Definition
def AH(ImC):
	Siz = ImCropTh[0]
	W = ImCropTh[1]
	#f = pylab.figure()
	a1 = ImC[(Siz/2):((Siz/2)+W),(Siz/2):(Siz/2)+W]
	a2 = ImC[(Siz/2):((Siz/2)+W),((Siz/2)-W):(Siz/2)]
	a3 = ImC[((Siz/2)-W):(Siz/2),((Siz/2)-W):(Siz/2)]
	a4 = ImC[((Siz/2)-W):(Siz/2),(Siz/2):((Siz/2)+W)]
	#f.add_subplot(2,2,1),pylab.imshow(a1,cmap=cm.Greys_r)
	#f.add_subplot(2,2,2),pylab.imshow(a2,cmap=cm.Greys_r)
	#f.add_subplot(2,2,3),pylab.imshow(a3,cmap=cm.Greys_r)
	#f.add_subplot(2,2,4),pylab.imshow(a4,cmap=cm.Greys_r)
	#pylab.show()

	p = np.zeros((4*W,W/2), dtype = 'uint16')
	pix = np.zeros((4*W,W/2), dtype = 'uint16')
	c1 = np.zeros((4*W,W/2), dtype = 'uint16')
	SUM = np.zeros(4*W, dtype = 'uint16')

	#First Quadrant
	for i in range(0,W/2):
		rat = float(i*2)/W
		ang = math.atan(rat)
		for j in range(0,W/2):
			p[i,j] = (j*2)*math.tan(ang)
			pix[i,j] = round(p[i,j])
			c1[i,j] = a1[pix[i,j],j*2]
	for i in range(W/2,W):
		rat = float((W-i)*2)/W
		ang = math.atan(rat)
		for j in range(0,W/2):
			p[i,j] = (j*2)*math.tan(ang)
			pix[i,j] = round(p[i,j])
			c1[i,j] = a1[j*2,pix[i,j]]

	#Second Quadrant
	for i in range(W,3*W/2):
		rat = float((i-W)*2)/W
		ang = math.atan(rat)
		for j in range(0,W/2):
			p[i,j] = (j*2)*math.tan(ang)
			pix[i,j] = round(p[i,j])
			c1[i,j] = a2[j*2,W-1-pix[i,j]]
	for i in range(3*W/2,2*W):
		rat = float((2*W-i)*2)/W
		ang = math.atan(rat)
		for j in range(0,W/2):
			p[i,j] = (j*2)*math.tan(ang)
			pix[i,j] = round(p[i,j])
			c1[i,j] = a2[pix[i,j],W-1-j*2]

	#Third Quadrant
	for i in range(2*W,5*W/2):
		rat = float((i-2*W)*2)/W
		ang = math.atan(rat)
		for j in range(0,W/2):
			p[i,j] = (j*2)*math.tan(ang)
			pix[i,j] = round(p[i,j])
			c1[i,j] = a3[W-1-pix[i,j],W-1-j*2]
	for i in range(5*W/2,3*W):
		rat = float((3*W-i)*2)/W
		ang = math.atan(rat)
		for j in range(0,W/2):
			p[i,j] = (j*2)*math.tan(ang)
			pix[i,j] = round(p[i,j])
			c1[i,j] = a3[W-1-j*2,W-1-pix[i,j]]

	#Fourth Quadrant
	for i in range(3*W,7*W/2):
		rat = float(i-3*W)/W
		ang = math.atan(rat)
		for j in range(0,W/2):
			p[i,j] = (j*2)*math.tan(ang)
			pix[i,j] = round(p[i,j])
			c1[i,j] = a4[W-1-j*2,pix[i,j]]
	for i in range(7*W/2,4*W):
		rat = float((4*W-i)*2)/W
		ang = math.atan(rat)
		for j in range(0,W/2):
			p[i,j] = (j*2)*math.tan(ang)
			pix[i,j] = round(p[i,j])
			c1[i,j] = a4[W-1-pix[i,j],j*2]

	for i in range(0,4*W):
		SUM[i] = sum(c1[i,:])
	return SUM
# End of AngularHistogram Definition

def FS(Ig,Xlm,Ylm,ImScale,SUMdb,RotAng,DeLms,n):
	global SUMTemp
	Siz = ImCropTh[0]
	W = ImCropTh[1]
	st_time = time.time()
	Ig = np.array(Ig)
	h,w = Ig.shape[:2]

	h = int(round(h*ImScale))
	w = int(round(w*ImScale))
	IgN = imresize(Ig,(h,w))
	h1,w1 = IgN.shape[:2]
	IgNCrop = np.zeros((Siz,Siz,n),dtype = 'uint8')
	for i in range(0,n):
		p1 = int(round(Xlm[i]*ImScale))
		p2 = int(round(Ylm[i]*ImScale))
		IgNCrop[:,:,i] = IgN[p2-(Siz/2):p2+(Siz/2),p1-(Siz/2):p1+(Siz/2)]
		#cv2.imshow('IgNCrop',IgNCrop[:,:,i])
		#cv2.waitKey(2500)
	#cv2.imwrite('croppedIm.png',IgNCrop[:,:,1])
	IgNRot= np.zeros((Siz,Siz,n),dtype = 'uint8')
	for i in range(0,n):
		IgNRot[:,:,i] = imrotate(IgNCrop[:,:,i],RotAng[i])
	SUM = np.zeros((n,4*W),dtype = 'uint16')
	st_ti_AnHi = time.time()
	for i in range(0,n):
		SUM[i,:] = AH(IgNRot[:,:,i])
	if DataGen == True:
		if n = NLdmrk
			SUM0 = SUM
			dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
			np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
		elif n == NLmrk-1:
			SUMTemp = SUM[:][0]
			SUM[:][0] = SUM
			dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
			np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
			
#	print '........................', time.time() - st_ti_AnHi, "Time required for performing Angular Histogram ...."	
	np.save('FeatSig.npy',SUM)	
	S1h = len(SUM)
	S1w = len(SUM[0])

	cor = np.zeros((S1h,n),dtype = 'float')
	cmax = np.zeros(S1h,dtype = 'float')
	cind = np.zeros(S1h,dtype = 'uint16')
	st_ti_corr = time.time()
	for i in range(0,S1h):
		for j in range(0,n):
			cor[i,j] = BasicFns.corr(SUMdb[i,:],SUM[j,:])
#	print '...............................', time.time() - st_ti_corr, "Time required for performing Correlation ...."
	print 'cor =', cor

	cmax = np.amax(cor,axis=0)
	cind = np.argmax(cor,axis=0)

	for i in range(0,S1h):
		if cmax[i] < 0.5:
			cind[i] = -1
	for i in range(0,S1h-1):
		for j in range(i+1,S1h):
			if cind[i] == cind[j]:
				if cmax[i] > cmax[j]:
					cind[j] = 0
				else:
					cind[i] = 0
	count = 0
	MatPairs = np.zeros((1,2),dtype = 'uint16')
	for i in range(0,S1h):
		if cind[i] >= 0:
			if count == 0:
				MatPairs = [[i,cind[i]]]
			else:
				MatPairs = np.append(MatPairs,[[i,cind[i]]],axis=0)
			count = count + 1
	print '....................................................', time.time() - st_time, "seconds for FeatSig.FS ...."		
	return MatPairs,SUM,cind,cor
# End of FeatureSignature Definition

def LL(LdmrkCen,LdmrkAngsDb):
	global LdmrkAngsCal0,LdmrkAngsCal1,LdmrkAngsTemp
	LdmrkCen = np.array(LdmrkCen)
	LdmrkAngsDb = np.array(LdmrkAngsDb)
	CenCal = BasicFns.Centroid(LdmrkCen)
	l = len(LdmrkCen)
	LdmrkAngsCal = np.zeros((l,l-1),dtype = 'float')
	DeLms = np.zeros(l,dtype = 'int8')
	Xlm = np.zeros(l,dtype = 'float')
	Ylm = np.zeros(l,dtype = 'float')
	RotAng = np.zeros(l,dtype = 'float')
	for i in range(0,l):
		count = 0
		for j in range(0,l):
			if i != j:
				LdmrkAngsCal[i,count] = BasicFns.Angle3Pt(LdmrkCen[i,0],LdmrkCen[i,1],CenCal[0],CenCal[1],LdmrkCen[j,0],LdmrkCen[j,1])
				count = count + 1
	LdmrkAngsCal = np.sort(LdmrkAngsCal)
	
	if GenData == True:
		if len(LdmrkAngsDb[0]) == NLdmrk-1:
			LdmrkAngs0 = LdmrkAngsCal
			dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
			np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
		elif len(LdmrkAngsDb[0]) == NLdmrk-2:
#			LdmrkAngsCal1 = LdmrkAngsCal
			LdmrkAngsTemp = LdmrkAngs1[:][0]
			LdmrkAngs1[:][0] = LdmrkAngsCal
			dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
			np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
	for i in range(0,l):
		for j in range(0,l):
			AngDiff = abs(np.subtract(LdmrkAngsCal[i,:],LdmrkAngsDb[j,:]))
			DeLms[i] = -1
			Xlm[i] = LdmrkCen[j,0]
			Ylm[i] = LdmrkCen[j,1]
			if np.all(AngDiff < LdmrkAngTh):
				DeLms[i] = j
				Xlm[i] = LdmrkCen[j,0]
				Ylm[i] = LdmrkCen[j,1]
				break
		RotAng[i] = BasicFns.Angle2Pt(Xlm[i],Ylm[i],CenCal[0],CenCal[1])		
#	print 'RotAng',RotAng	
#	print 'DetectedLdmrks',DeLms
	return CenCal,Xlm,Ylm,DeLms,RotAng
# End of LdmrkLoc Definition

def LD(Ig):
	x = -1
	k = -1
	global BinaryTh, BinThIncDec, SoSTemp
	PmFlag = True						# modified codes begin here (20/4/15)
	BinCounter = 0
	h,w = Ig.shape[:2]
#	cv2.imshow('Grayscale Image',Ig)
#	cv2.waitKey(2000)
	while 1:
		_,Ib = cv2.threshold(Ig, BinaryTh, 1, cv2.THRESH_BINARY)
		cv2.imshow('Binary Image',Ib*255)
		cv2.waitKey(200)
		SumBin = sum(sum(Ib))
#		print 'SumBin =',SumBin
		BinImFactor = float(SumBin)/(h*w) 
#		print 'BinImFactor =',BinImFactor
#		print 'BinaryTh =',BinaryTh
		if BinImFactor > ImageTh and BinImFactor < ImageTh*2:
			break
		if BinImFactor < ImageTh:
			if PmFlag == True:
				BinThIncDec = -BinThIncDec/2
			else:
				BinThIncDec = BinThIncDec	
			PmFlag = False
			BinCounter = BinCounter + 1
		elif BinImFactor > ImageTh*1.5:
			if PmFlag == False:
				BinThIncDec = -BinThIncDec/2
				PmFlag = True
				BinCounter = BinCounter + 1
		if BinCounter > 4:
			break
		BinaryTh = BinaryTh + BinThIncDec			
	cv2.imshow('Binary Image',Ib*255)
	cv2.waitKey(1000)
			
	#modified codes end here (20/4/15)											
#	print 'BinImFactor after first loop=',BinImFactor
	NumCanLdmrk = NLdmrk + 1 						#Initialization of number of candidate landmarks
	while ((NumCanLdmrk > NLdmrk) or (BinImFactor > ImageTh)):
		_,Ib = cv2.threshold(Ig, BinaryTh, 1, cv2.THRESH_BINARY)
		element = cv2.getStructuringElement(cv2.MORPH_CROSS,(4,4))
		Ib = cv2.erode(Ib,element);
		cv2.imshow('Binary Image after erosion',Ib*255)
		cv2.waitKey(2000)
		_,cs,_ = cv2.findContours(Ib.astype('uint8'), mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE )
		BinaryTh = BinaryTh + 1
#		print 'BinaryTh =',BinaryTh
		NumCanLdmrk = len(cs)
		SumBin = sum(sum(Ib))
		BinImFactor = float(SumBin)/(h*w)
		for i in range(0,NumCanLdmrk):		#This for loop is for "BWAreaOpen"
			c = cs[i]
			m = cv2.moments(c)
			Area = m['m00']
			if Area <= BwAreaTh[0] or Area >= BwAreaTh[1]:
				cv2.drawContours(Ib, cs, i, (0,0,0), -1)
		_,cs,_ = cv2.findContours(Ib.astype('uint8'), mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
		NumCanLdmrk = len(cs)
	CanLdmrkCen = []
#	print 'BinImFactor Final =',BinImFactor
	cv2.imshow('Binary Image',Ib*255)
	cv2.waitKey(2000)	
	print 'NumCanLdmrk=',NumCanLdmrk
	for i in range(0,NumCanLdmrk):
		c = cs[i]
		m = cv2.moments(c)
		CanLdmrkCen.append([])
		CanLdmrkCen[i].append(m['m10']/m['m00'])	
		CanLdmrkCen[i].append(m['m01']/m['m00'])
#	print "CanLdmrkCen",CanLdmrkCen
	BrFlag = False
	DetFlag = False
	PosLdmrks = []
	SUM = []
	MatPairs = []
#	cv2.imshow('Binary Image',Ib)
#	cv2.waitKey(5000)
	
	if NumCanLdmrk > NLdmrk:
#		print "Candidate Landmarks (> desired)",NumCanLdmrk
		nums = np.arange(NumCanLdmrk) 		# nums = {1,2,...,Num_Can_Ldmrk}.
		combs = set(itertools.combinations(nums,NLdmrk))
		for i in range(0,len(combs)):
			PosLdmrks = list(combs)[i]			# Possible Landmarks
			LdmrkCen = np.zeros((len(PosLdmrks),2), dtype='float')
			for j in range(0,len(PosLdmrks)):
				LdmrkCen[j,:] = CanLdmrkCen[PosLdmrks[j]] 	
			CenCal,Xlm,Ylm,DeLms,RotAng = LL(LdmrkCen,Data[0][0]) 	# Data[0] = Ldmrk_Angs
			if NonNegEls == NLdmrk:
				SoS = BasicFns.SumSides(Xlm,Ylm)
				if GenData == True:
					SoS0 = SoS
					dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
					np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
#				print 'SoS',SoS
				ImScale  = Data[1][0]/SoS			# Data[1] = Ldmrk_Sides_Sum
				MatPairs,SUM,cind,cor = FS(Ig,Xlm,Ylm,ImScale,Data[2][0],RotAng,DeLms,NumCanLdmrk)	# Data[2] = Feat_Sigs
				NonNegEls1 = MatPairs[(MatPairs >= 0)].size
				if NonNegEls1 == 8:
					DetFlag = True
					print "Successful Detection of all", NLdmrk ,"Landmarks"
					BrFlag = True
					break
				
		if DetFlag == False:
			print "Can not match", NLdmrk, "Landmarks: Now will try to match", NLdmrk-1, "Landmarks"				
			for k in range(NLdmrk-1,2,-1):
				combs = set(itertools.combinations(nums,k))
				LdmrkCen = np.zeros((k,2), dtype='float')
				for i in range(0,len(combs)):
					PosLdmrks = list(combs)[i]	# Possible Landmarks
					for j in range(0,len(PosLdmrks)):
						LdmrkCen[j,:] = CanLdmrkCen[PosLdmrks[j]]
					for x in range(0,len(Data[0][NLdmrk-k])):
						CenCal,Xlm,Ylm,DeLms,RotAng = LL(LdmrkCen,Data[0][NLdmrk-k][x]) 
						DeLms = np.array(DeLms)
						NonNegEls = DeLms[(DeLms >= 0)].size
						if NonNegEls == k:
							SoS = BasicFns.SumSides(Xlm,Ylm)
							if GenData == True:
								SoSTemp = SoS[0]
								SoS1[0] = SoS
								dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
								np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
#							print 'SoS',SoS
							ImScale = float(Data[1][NLdmrk-k][x])/SoS
							MatPairs,SUM,cind,cor = FS(Ig,Xlm,Ylm,ImScale,Data[2][NLdmrk-k][x],RotAng,DeLms,k)
							NonNegEls1 = MatPairs[(MatPairs >= 0)].size
							if NonNegEls1 == k*2:
								print "Successful Detection of", k ,"landmarks",BasicFns.WhichLdmrkDeted(NumCanLdmrk,NLdmrk,k,x)
								BrFlag = True
								break
					if BrFlag == True:
						break
				if BrFlag == True:
					break	
				if k == 3:
					print "Landmarks could not be matched. Not even THREE Landmarks were matched"
		
	elif NumCanLdmrk == NLdmrk:
		print "Candidate Landmarks (= desired):",NumCanLdmrk
		LdmrkCen = CanLdmrkCen
		CenCal,Xlm,Ylm,DeLms,RotAng = LL(LdmrkCen,Data[0][0]) 	# Data[0] = Ldmrk_Angs
		DeLms = np.array(DeLms)
		NonNegEls = DeLms[(DeLms >= 0)].size
		if NonNegEls == NLdmrk:
			SoS = BasicFns.SumSides(Xlm,Ylm)
			if GenData == True:
				SoS0 = SoS
				dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
				np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
#			print 'SoS',SoS
			ImScale  = Data[1][0]/SoS		# Data[1] = Ldmrk_Sides_Sum
			PosLdmrks = np.arange(NLdmrk)
			MatPairs,SUM,cind,cor = FS(Ig,Xlm,Ylm,ImScale,Data[2][0],RotAng,DeLms,NumCanLdmrk) # Data[2] = Feat_Sigs
			NonNegEls1 = MatPairs[(MatPairs >= 0)].size
			if NonNegEls1 == 8:
				DetFlag = True
				print "Successful Detection of all", NLdmrk, "Landmarks"
		if DetFlag == False:
			print "Can not match", NLdmrk, "Landmarks: Now will try to match", NLdmrk-1, "Landmarks"
			nums = np.arange(NumCanLdmrk)
			for k in range(NLdmrk-1,2,-1):
				combs = set(itertools.combinations(nums,k))
				for i in range(0,len(combs)):
					PosLdmrks = list(combs)[i]			# Possible Landmarks
					LdmrkCen = np.zeros((len(PosLdmrks),2), dtype='float')
					for j in range(0,len(PosLdmrks)):
						LdmrkCen[j,:] = CanLdmrkCen[PosLdmrks[j]]
					for x in range(0,len(Data[0][NLdmrk-k])):
						CenCal,Xlm,Ylm,DeLms,RotAng = LL(LdmrkCen,Data[0][NLdmrk-k][x]) 
						DeLms = np.array(DeLms)
						NonNegEls = DeLms[(DeLms >= 0)].size
						if NonNegEls == k:
							SoS = BasicFns.SumSides(Xlm,Ylm)
							if GenData == True:
								SoSTemp = SoS[0]
								SoS1[0] = SoS
								dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
								np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
#							print 'SoS',SoS
							ImScale = float(Data[1][NLdmrk-k][x])/SoS
							MatPairs,SUM,cind,cor = FS(Ig,Xlm,Ylm,ImScale,Data[2][NLdmrk-k][x],RotAng,DeLms,k)
							NonNegEls1 = MatPairs[(MatPairs >= 0)].size
							if NonNegEls1 == k*2:
								print "Successful Detection of", k ,"landmarks",BasicFns.WhichLdmrkDeted(NumCanLdmrk,NLdmrk,k,x)
								BrFlag = True
								break
					if BrFlag == True:
						break
				if BrFlag == True:
					break	
				if k == 3:
					print "Landmarks could not be matched. Not even THREE Landmarks were matched"

	else:
		print "Candidate Landmarks (< desired):",NumCanLdmrk
		nums = np.arange(NumCanLdmrk)
		for k in range(NumCanLdmrk,2,-1):
			combs = set(itertools.combinations(nums,k))
			print 'combs',combs
			for i in range(0,len(combs)):
				PosLdmrks = list(combs)[i]					# Possible Landmarks
				LdmrkCen = np.zeros((len(PosLdmrks),2), dtype='float')
				for j in range(0,len(PosLdmrks)):
					LdmrkCen[j,:] = CanLdmrkCen[PosLdmrks[j]]
				for x in range(0,len(Data[0][NLdmrk-k])):
					CenCal,Xlm,Ylm,DeLms,RotAng = LL(LdmrkCen,Data[0][NLdmrk-k][x]) 
					DeLms = np.array(DeLms)
					NonNegEls = DeLms[(DeLms >= 0)].size
					if NonNegEls == k:
						SoS = BasicFns.SumSides(Xlm,Ylm)
						if GenData == True:
							SoSTemp = SoS[0]
							SoS1[0] = SoS
							dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
							np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
#						print 'SoS',SoS
						ImScale = float(Data[1][NLdmrk-k][x])/SoS
						MatPairs,SUM,cind,cor = FS(Ig,Xlm,Ylm,ImScale,Data[2][NLdmrk-k][x],RotAng,DeLms,k)
						NonNegEls1 = MatPairs[(MatPairs >= 0)].size
						if NonNegEls1 == k*2:
							print "Successful Detection of", k ,"landmarks",BasicFns.WhichLdmrkDeted(NumCanLdmrk,NLdmrk,k,x)
							BrFlag = True
							break
				if BrFlag == True:
					break
			if BrFlag == True:
				break	
			if k == 3:
				print "Landmarks could not be matched. Not even THREE Landmarks were matched"	
	if (NLdmrk != NumCanLdmrk) or ((NLdmrk == NumCanLdmrk) and (len(PosLdmrks) < NLdmrk)):
		MatPairs[:,0] = PosLdmrks	
		MatPairs[:,1] = DeLms
	else:
		MatPairs[:,0] = DeLms	
		MatPairs[:,1] = cind
#	print 'ImScale',ImScale
	return Ib,cor,Xlm,Ylm,CenCal,CanLdmrkCen,MatPairs,SUM,ImScale,SoS,x,k
# End of LdmrkDet Definition	

def WD(Im,Xlm,Ylm,x,k):
	global WpAngsTemp,WpSideRatTemp
	wpDetected = False
	WpAngsCal,WpSideRatCal = BasicFns.PolyAngSid(Xlm,Ylm)
	if x == -1:
		if DataGen == True:
			WpAngs0 = WpAngsCal
			WpSideRat0 = WpSideRatCal
			dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
			np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
		AngDiff = abs(Data[3][0] - WpAngsCal)
		SideDiff = abs(Data[4][0] - WpSideRatCal) 
	else:
		if DataGen == True:
			WpAngsTemp = WpAngs1[:][0]
			WpSideRatTemp = WpSideRat1[:][0]
			WpAngs1[:][0] = WpAngsCal
			WpSideRat1[:][0] = WpSideRatCal
			dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
			np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
		AngDiff = abs(Data[NLdmrk-k][x] - WpAngsCal)
		SideDiff = abs(Data[NLdmrk-k][x] - WpSideRatCal)
#	print 'AngDiff',AngDiff
#	print 'SideDiff',SideDiff
	if np.all(AngDiff < WpAngTh) & np.all(SideDiff < WpSiRatTh):
			wpDetected = True
	return wpDetected,WpAngsCal,WpSideRatCal
# End of WpDet Definition		

def GD(curWpNo,dataWritten):
	dataToWrite = [[LdmrkAngs0,[LdmrkAngs1]],[SoS0,[SoS1]],[SUM0,[SUM1]],[WpAngs0,[WpAngs1]],[WpSideRat0,[WpSideRat1]],[RefPt0,[RefPt1]],AltRef0,YawRef0]
	np.save('Database/Wp'+str(curWpNo)+'Data.npy',dataToWrite)
	dataWritten = True
#	print 'dataToWrite = ',dataToWrite
	return dataWritten
	

def IP(Im,Yaw,AltRef,YawRef,curWpNo,GenData,parLdmrkNos):
	global LdmrkAngs0,LdmrkAngs1,SoS0,SoS1,SUM0,SUM1,WpAngs0,WpAngs1,WpSideRat0,WpSideRat1,RefPt0,RefPt1
	IpCalDrift = np.array(2,dtype = 'float')
	h,w = Im.shape[:2]
	ImgCen = [w/2,h/2]
	Ig = cv2.cvtColor(Im, cv2.COLOR_RGB2GRAY)	
	
	Ib,cor,Xlm,Ylm,CenCal,CanLdmrkCen,MatPairs,SUM,ImScale,SoS,x,k = LD(Ig)

	for i in range(0,len(Xlm)):
		Xlm[i] = CanLdmrkCen[LdmrkSeq[i]][0]
		Ylm[i] = CanLdmrkCen[LdmrkSeq[i]][1]
	ldmrkCentroids = np.concatenate([Xlm[np.newaxis,:],Ylm[np.newaxis,:]],axis = 0)		
	ldmrkCentroids = ldmrkCentroids.T

	refptCal = BasicFns.Centroid(ldmrkCentroids)

	WayPt,WpAngsCal,WpSideRatCal = WD(Im,Xlm,Ylm,x,k)	

	if len(LdmrkSeq) == NLdmrk:
		SoS0 = SoS
		SUM0 = SUM
		WpAngs0 = WpAngsCal
		WpSideRat0 = WpSideRatCal
		RefPt0 = refptCal

		print 'LdmrkAngs = ',LdmrkAngs0
		print 'SoS = ',SoS0
		print 'WpAngs = ',WpAngs0
		print 'WpSideRat = ',WpSideRat0
		print 'RefPt = ',RefPt0

	MatPairs = MatPairs[np.lexsort((MatPairs[:,1], ))]	
	LdmrkSeq = MatPairs[:,0]
	print 'LdmrkSeq', LdmrkSeq
	if len(LdmrkSeq) == NLdmrk-1:
		if LdmrkSeq == [0, 1, 2]:
			i = 0
		elif LdmrkSeq == [0, 1, 3]:
			i = 1
		elif LdmrkSeq == [0, 2, 3]:
			i = 2
		elif LdmrkSeq == [2, 3, 4]:
			i = 3
		LdmrkAngs1[:][i] = LdmrkAngsCal1
		SoS1[i] = SoS
		SUM1[:][i] = SUM
		WpAngs1[:][i] = WpAngsCal
		WpSideRat1[:][i] = WpSideRatCal
		RefPt1[:][i] = refptCal	

		print 'LdmrkAngs = ',LdmrkAngs1[:][i]
		print 'SoS = ',SoS1[i]
		print 'WpAngs = ',WpAngs1[:][i]
		print 'WpSideRat = ',WpSideRat1[:][i]
		print 'RefPt = ',RefPt1[:][i]


	rpcalToXY = BasicFns.ToXyPlane(refptCal,ImgCen,Yaw)
	
	if x == -1:		# This condition is active when all landmarks in DB are detected.
		refptDB = Data[5][0]
	else:			# This condition is active when fewer landmarks are detected.
		refptDB = Data[5][NLdmrk-k][x]	
	rpdbToXY = BasicFns.ToXyPlane(refptDB,ImgCen,Data[6])
	refptDiffXY = list(imap(sub,rpdbToXY,rpcalToXY)) 	

	if GenData == True:
		AltRef0 = AltRef
		YawRef0 = YawRef
		dataWritten = False
		dataWritten = GD(curWpNo,dataWritten)
		if (dataWritten):
			print 'Data has been scuccessfully written to file'
		else:
			print 'Data write ERROR: There was problem in writing data'
	AltRef = Data[6]
	YawRef = Data[7]
	IpCalDrift = [(0.48/400)*(AltRef)*(ImScale)*k for k in refptDiffXY]
	print 'IpCalDrift = ',IpCalDrift
	return IpCalDrift
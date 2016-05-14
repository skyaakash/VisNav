import numpy as np
import math
import itertools

cd2r = (math.pi)/18000
d2r = (math.pi)/180
r2d = (1/d2r)
r2cd =(1/cd2r)
# corr Definition
def corr(a,b):
	ma = np.mean(a)
	mb = np.mean(b)
	la = len(a)
	Da = np.zeros(la)
	Db = np.zeros(la)
	DaMb = np.zeros(la)
	DaSq = np.zeros(la)
	DbSq = np.zeros(la)
	for i in range(0,la):
		Da[i] = a[i] - ma
		Db[i] = b[i] - mb
		DaMb[i] = float(Da[i])*Db[i]
		DaSq[i] = pow(Da[i],2)
		DbSq[i] = pow(Db[i],2)

	Num = sum(DaMb)
	Den = np.sqrt(sum(DaSq)*sum(DbSq))
	cor = float(Num)/Den
	return cor
# End of corr Definition

# Centroid Definition
def Centroid(LdmrkCen):
	cen1 = 0
	cen2 = 0
	for i in range(0,len(LdmrkCen)):
		cen1 = cen1 + LdmrkCen[i,0]
		cen2 = cen2 + LdmrkCen[i,1]

	cen1 = float(cen1)/(len(LdmrkCen))
	cen2 = float(cen2)/(len(LdmrkCen))
	cen = [cen1,cen2]
	return cen
# End of Centroid Definition

# SumSides Definition
def SumSides(Xllm,Yllm):
	l = len(Xllm)
	Xlm = np.zeros(l+1, dtype = 'float')
	Ylm = np.zeros(l+1, dtype = 'float')
	Xlm[0:l] = Xllm
	Ylm[0:l] = Yllm
	Xlm[l] = Xllm[0]
	Ylm[l] = Yllm[0]
	SoS = 0
	for i in range(0,l):
		side = math.sqrt(pow((Xlm[i+1]-Xlm[i]),2) + pow((Ylm[i+1]-Ylm[i]),2))
		SoS = SoS + side
	return SoS
# End of SumSides Definition

# Angle3Pt Definition
def Angle3Pt(p1x,p1y,p2x,p2y,p3x,p3y):
	line1 = [p2x,p2y,p1x-p2x,p1y-p2y]
	line2 = [p2x,p2y,p3x-p2x,p3y-p2y]
	theta1 = np.mod(math.atan2(line1[3],line1[2]) + 2*math.pi, 2*math.pi)
	theta2 = np.mod(math.atan2(line2[3],line2[2]) + 2*math.pi, 2*math.pi)
	theta = np.mod((theta2-theta1) + 2*math.pi, 2*math.pi)
	theta = theta*r2d
	return theta	
# End of Angle3Pt Definition

# Angle2Pt Definition
def Angle2Pt(a,b,c,d):
	xdiff = c - a
	ydiff = d - b
	deg = math.degrees(math.atan2(ydiff,xdiff))
	return deg
# End of Angle2Pt Definition

# WhichLdmrkDeted Definition
def WhichLdmrkDeted(NumCanLdmrk,NLdmrk,k,x):
	nums = np.arange(NLdmrk)
	combs = set(itertools.combinations(nums,k))
	DetedLdmrks = list(combs)[x]
	return DetedLdmrks
# End of WhichLdmrkDeted Definition

# PolyAngSid Definition
def PolyAngSid(XLM,YLM):
	l = len(XLM)
	points = np.zeros((l,2), dtype = 'float')
	points[:,0] = XLM
	points[:,1] = YLM
	
	Ang = np.zeros(l, dtype = 'float')
	Side = np.zeros(l, dtype = 'float')
	for i in range(len(points)):
		p1 = points[i]
		ref = points[i - 1]
		p2 = points[i - 2]
		x1, y1 = p1[0] - ref[0], p1[1] - ref[1]
		x2, y2 = p2[0] - ref[0], p2[1] - ref[1]
		Side[i] = math.sqrt(pow((p1[0]-ref[0]),2) + pow((p1[1]-ref[1]),2))
		Ang[i] = Angle(x1, y1, x2, y2)*180/math.pi
#		if CrossSign(x1, y1, x2, y2):
#			print('Inner Angle')
#		else:
			#Ang[i] = 360 - Ang[i]
#			print('Outer Angle')
	SideRat = Side/sum(Side)
	return Ang,SideRat
# End of PolyAngSid Definition

# Definitions to support PolyAngSid
def Angle(x1, y1, x2, y2):
    # Use dotproduct to find angle between vectors
    # This always returns an angle between 0, pi
    numer = (x1 * x2 + y1 * y2)
    denom = math.sqrt((x1 ** 2 + y1 ** 2) * (x2 ** 2 + y2 ** 2))
    return math.acos(float(numer) / denom) 
def CrossSign(x1, y1, x2, y2):
    # True if cross is positive
    # False if negative or zero
    return x1 * y2 > x2 * y1
# End of Definitions to support PolyAngSid

# Pol2Cart Definition
def Pol2Cart(Mag,AngDeg):
	AngRad = AngDeg*d2r
	Vb = np.zeros(2,dtype = 'float')
	Vb[0] = Mag*math.cos(AngRad)
	Vb[1] = Mag*math.sin(AngRad)
	return Vb	
# End of Pol2Cart Definition

# Cart2Pol Definition
def Cart2Pol(Num):
	Mag = math.sqrt(pow(Num[0],2) + pow(Num[1],2))
	if Num[0] == 0:
		if Num[1] > 0:
			AngRad = math.pi/2
		else:
			AngRad = (3/2)*math.pi 
	else:
		AngRad = math.atan(Num[1]/Num[0])
		if Num[0] < 0:
				AngRad = AngRad + math.pi
	AngDeg = AngRad*180/math.pi
	return Mag,AngDeg	
# End of Cart2Pol Definition

# Euler2Cartesian Definition
def Euler2Cartesian(Mag,Pitch,Yaw):
	X = Mag*math.cos(Pitch*d2r)*math.cos(Yaw*d2r)
	Y = Mag*math.cos(Pitch*(math.pi/180))*math.sin(Yaw*d2r)
	Z = Mag*math.sin(Pitch*d2r)
	return X,Y,Z
# End of Euler2Cartesian Definition

# DistanceOnUnitSphere Definitiion
def DistanceOnUnitSphere(lat1,lon1,lat2,lon2):
	phi1 = (90 - lat1)*d2r
	phi2 = (90 - lat2)*d2r

	theta1 = lon1*d2r
	theta2 = lon2*d2r

	cos = (math.sin(phi1)*math.sin(phi2)*math.cos(theta1 - theta2) + math.cos(phi1)*math.cos(phi2))
	arc = math.acos(cos)

	return arc
# End of DistanceOnUnitSphere Definitiion

# SubtractVectors Definition
def SubtractVectors(Vec1Mag,Vec1Dir,Vec2Mag,Vec2Dir):
	Vec1 = [0,0]
	Vec2 = [0,0]
	Vec = [0,0]
	Vec1[0] = Vec1Mag*math.cos(Vec1Dir*cd2r)
	Vec1[1] = Vec1Mag*math.sin(Vec1Dir*cd2r)
	Vec2[0] = Vec2Mag*math.cos(Vec2Dir*d2r)
	Vec2[1] = Vec2Mag*math.sin(Vec2Dir*d2r)

	Vec[0] = Vec1[0] - Vec2[0]
	Vec[1] = Vec1[1] - Vec2[1]

	VecMag = math.sqrt(Vec[0]*Vec[0] + Vec[1]*Vec[1])
	if Vec[0] == 0:
		if Vec[1] < 0:
			VecDir = 270
		else:
			VecDir = 90
	else:
		VecDir = (180/math.pi)*math.atan(Vec[1]/Vec[0])

	if Vec[0] < 0:
		VecDir = VecDir + 180
	else:
		if Vec[1] < 0:
			VecDir = VecDir + 360	
	VecDir = VecDir*100		
	return VecMag,VecDir
# End of SubtractVectors Definition

# YawCvelCalc Definition
def YawCvelCalc(Asp,CAng,WindM,WindD):
	Yaw = np.zeros(1,dtype = 'float')	
	CVel = np.zeros(1,dtype = 'float')
	Yaw = CAng + math.asin((float(WindM)/Asp)*math.sin(WindD*d2r-CAng*cd2r))*r2cd	
	if Yaw > 36000:
		Yaw = Yaw - 36000
	elif Yaw < 0:
		Yaw = Yaw + 36000		
	if CAng == 9000 or CAng == 27000:
		CVel = (Asp*math.sin(Yaw*cd2r) - WindM*math.sin(WindD*d2r))/(math.sin(CAng*cd2r))
	else:
		CVel = (Asp*math.cos(Yaw*cd2r) - WindM*math.cos(WindD*d2r))/(math.cos(CAng*cd2r))		
	return CVel,Yaw
# End of YawCvelCalc Definition

# AspCAngCalc Definition
def AspCAngCalc(Vg,Yaw,WindM,WindD):
#	A = abs((Yaw/100 + WindD)*d2r)
#	print 'A =',A
#	C = math.asin((WindM/Vg)*math.sin(A))
#	print 'C =',C
#	B = math.pi - A - C
#	print 'B =',B
#	Asp = (math.sin(B)/math.sin(A))*Vg
#	Yaw = 9000 - Yaw
#	if Yaw < 0:
#		Yaw = Yaw + 36000
#	print 'Yaw =',Yaw		
#	WindD = 90 - WindD
#	if WindD < 0: 
#		WindD = WindD + 360	
#	print 'WindD =',WindD	
	Asp = np.zeros(1,dtype = 'float')	
	CAng = np.zeros(1,dtype = 'float')	
	CAng = Yaw - math.asin((float(WindM)/Vg)*math.sin(WindD*d2r-Yaw*cd2r))*r2cd
	if CAng > 36000:
		CAng = CAng - 36000
	elif CAng	< 0:
		CAng = CAng + 36000
	if Yaw == 9000 or Yaw == 27000:
		Asp = (Vg*(math.sin(CAng*cd2r)) + WindM*(math.sin(WindD*d2r)))/(math.sin(Yaw*cd2r))
	else:
		Asp = (Vg*(math.cos(CAng*cd2r)) + WindM*(math.cos(WindD*d2r)))/(math.cos(Yaw*cd2r))		
	return Asp,CAng
# End of AspCangCalc Definition

# HeadAngFromLatLon Definition
def HeadAngFromLatLon(lat1,lon1,lat2,lon2):
	lat1 = lat1*d2r
	lon1 = lon1*d2r
	lat2 = lat2*d2r
	lon2 = lon2*d2r
	dLon = (lon2 - lon1)
	y = math.sin(dLon) * math.cos(lat2)
	x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
	HAng = math.atan2(y, x)
	HAng = HAng*(180/(math.pi))
	if HAng < 0:
		HAng = 360 + HAng
	return HAng*100
# End of HeadAngFromLatLon Definition	

# ToXyPlane Definition
def ToXyPlane(CurCen,ImgCen,CurYaw):
	[X1,Y1] = ImgCen
	[X2,Y2] = CurCen
	RefV_m = math.sqrt((X1-X2)**2 + (Y1-Y2)**2)
	if X1 == X2:
		if Y2 > Y1:
			VecAng = 270
		else:
			VecAng = 90	
	else:
		VecAng = math.atan((Y1-Y2)/(X2-X1))*(1/d2r)
		if X1 > X2:		# Second and Third Quadrants
			VecAng = VecAng + 180
		else:
			if Y2 > Y1: # Fourth Quadrant
				VecAng = VecAng + 360	
	XY_ang = VecAng - CurYaw
	if XY_ang < 0:
		XY_ang = XY_ang + 360
	CenXy = [RefV_m*math.cos(XY_ang*d2r),RefV_m*math.sin(XY_ang*d2r)]
	return CenXy
# End of ToXyPlane Definition	

# Test DistanceOnUnitSphere
#lat1 = -34.396672
#lon1 = 138.534130
#lat2 = -34.399760
#lon2 = 138.532638
#arc = DistanceOnUnitSphere(lat1,lon1,lat2,lon2)
#dist = arc*6373000
#print 'dist = ', dist
# End of Test DistanceOnUnitSphere

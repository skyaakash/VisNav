import math
X1 = 0
Y1 = 0
X2 = -1
Y2 = 0
cd2r = math.pi/18000
d2r = math.pi/180
r2d = 180/math.pi
r2cd = 18000/math.pi

if X1 == X2: # ReqYaw Calculations
	if Y2 > Y1:
		ReqXyAng = 9000
	else:
		ReqXyAng = 27000
else: 
	ReqXyAng = math.atan((Y2-Y1)/(X2-X1))
	ReqXyAng = ReqXyAng*r2cd

	if X2 < X1:
		ReqXyAng = ReqXyAng + 18000
	else:
		if Y2 < Y1:
			ReqXyAng = ReqXyAng + 36000	
ReqYaw = ReqXyAng - 9000
ReqYaw = 36000 - ReqYaw
if ReqYaw >=36000:
	ReqYaw = ReqYaw - 36000
print 'ReqXyAng = ',ReqXyAng
print 'ReqYaw = ',ReqYaw

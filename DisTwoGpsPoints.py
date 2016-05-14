import BasicFns
import math
Lat2 = -344251360
Lng2 = 1385243648
Lat1 = -344249458#-344222749
Lng1 = 1385246124#1385287571
cd2r = math.pi/18000

arc = BasicFns.DistanceOnUnitSphere(Lat1/1e7,Lng1/1e7,Lat2/1e7,Lng2/1e7)
DistBetPts = arc*6373000	
print 'DistBetPts =',DistBetPts

AngBetPts = BasicFns.HeadAngFromLatLon(Lat1/1e7,Lng1/1e7,Lat2/1e7,Lng2/1e7)
print 'AngBetPts =',AngBetPts

XyPlaneAng = 9000 - AngBetPts
if XyPlaneAng < 0:
	XyPlaneAng = XyPlaneAng + 36000
print 'XyPlaneAng =',XyPlaneAng	

Drift = [DistBetPts*math.cos(XyPlaneAng*cd2r),DistBetPts*math.sin(XyPlaneAng*cd2r)]
print 'Drift =',Drift

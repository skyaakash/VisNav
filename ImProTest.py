import cv2
import numpy as np
# Image processing Thresholds
BwAreaTh = np.array(2,dtype = 'float')
NLdmrk = [4]							# Number of candidate Landmarks to be detected
ImageTh = 0.004 						# Used in LdmrkDet.LD: To prevent excessive whites in binary image
BinaryTh = 110							# Initial binary threshold to  adaptively binarize an image
BinThIncDec = 5							# Increase or decrease binary threshold from intitial value (BinaryTh) (used in Algorithm 1)
BwAreaTh = [200,2000] 					# Lower and upper thresholds for BwArea
LdmrkAngTh = 5							# Threshold of Second Stage Landmark Detection
ImCropTh = [100,36]
WpAngTh = 5								# Threshold of Waypoint Angles
WpSiRatTh = 0.1							# Threshold of Waypoint Side Ratio

#End of Image processing Thresholds
AltRef = 400
YawRef = 207.71 

GenData = False							# Genarate Data
curWpNo = 1
parLdmrkNos = '123'	#012 013 023 123 	# Used to generate data with partial landmarks
Im = cv2.imread('Database/waypoint_'+str(curWpNo)+ '_'+parLdmrkNos+'.jpg')
# End of Image processing Thresholds

# Create Threshold Array
Thresholds = [NLdmrk,ImageTh,BinaryTh,BinThIncDec,BwAreaTh,LdmrkAngTh,ImCropTh, WpAngTh,WpSiRatTh]
np.save('Database/Wp'+str(curWpNo)+'Thresholds.npy',Thresholds)
print 'Thresholds = ',Thresholds
# End of Create Threshold Array
Yaw = YawRef
import ImageProcessing
ImageProcessing.LoadData(curWpNo,GenData,parLdmrkNos)
IpCalDrift = ImageProcessing.IP(Im,Yaw,AltRef,YawRef,curWpNo,GenData,parLdmrkNos)

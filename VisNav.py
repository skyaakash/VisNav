import FlightTest
totTrials = 1
useRPi = False
HiL = False
ImgPro = False				# Use Image Processing Correction at Waypoints 
AutoWpTh = 10				#180			#For Mallala and Woomera Tests
#HeadErr = np.loadtxt('/home/pi/FlightTest/Database/HeadErr1.txt')
HeadErr = 0
RollCdMax = 4500			# Not used in SIM
TiOutTh = 2					# Timeout Threshold for serial port (seconds)

FlightTest.main(useRPi,HiL,totTrials,ImgPro,AutoWpTh,HeadErr,RollCdMax,TiOutTh)

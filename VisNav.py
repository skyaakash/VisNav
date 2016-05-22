import FlightTest
totTrials = 1				# Number of trials to be contducted. (With this field, multiple trails can be conducted. More useful in Flight Tests). Set to 1 if single trial is used.
useRPi = False				# Use Raspberry Pi Flag. If FALSE, simultor will be used
HiL = False					# Hardawre in the Loop Flag. if(useRPi = TRUE and HiL = TRUE) => 'FLIGHT TEST'; elif(useRPi = TRUE and HiL = FALSE) => 'HIL TEST'; else => 'SIMULATOR'
ImgPro = False				# Use Image Processing Correction at Waypoints. if(TRUE) => 'correction will be used'; else => 'correction won't be used'
AutoWpTh = 10				#180			#For Mallala and Woomera Tests
#HeadErr = np.loadtxt('/home/pi/FlightTest/Database/HeadErr1.txt')
HeadErr = 0					# Heading error of compass. When used, Each autopilot should be calibrated and a Look up Table should be generated for each 0 to 360 degrees.
RollCdMax = 0			# Peak Roll angle during Turn. Not used in SIM
TiOutTh = 2					# Timeout Threshold for serial port (seconds). Not used in SIM

AutoWps = [] #[[-309353065,1365454102],[-309298649,1365464783],[-309315224,1365422668]]#Wo Auto Wps 28/4/15:0952 Wps 2,4 and 6 of WoomeraWps1
VisualWpsSim = [[0,50],[50,50],[50,0]]
VisualWps = [[-344249458,1385246124],[-344184456,1385176544],[-344140892,1385357971]]#[[-344249458,1385246124],[-344249458,1385286124]]
if useRPi == False:
	VisualWps = VisualWpsSim

FlightTest.main(useRPi,HiL,totTrials,ImgPro,AutoWpTh,HeadErr,RollCdMax,TiOutTh,AutoWps,VisualWps)

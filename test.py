import numpy as np
NLdmrk = 4
#LdmrkAngs = np.array
LdmrkAngs0 = [[0.0 for x in range(NLdmrk)] for y in range (NLdmrk-1)]
LdmrkAngs10 = [[0.0 for x in range(NLdmrk-1)] for y in range (NLdmrk-2)]
LdmrkAngs11 = [[0.0 for x in range(NLdmrk-1)] for y in range (NLdmrk-2)]
LdmrkAngs12 = [[0.0 for x in range(NLdmrk-1)] for y in range (NLdmrk-2)]
LdmrkAngs13 = [[0.0 for x in range(NLdmrk-1)] for y in range (NLdmrk-2)]
LdmrkAngs = [LdmrkAngs0,[LdmrkAngs10,LdmrkAngs11,LdmrkAngs12,LdmrkAngs13]]
#LdmrkAngs = np.array(LdmrkAngs)
print LdmrkAngs
#with open('testfile.txt', 'w') as f:
#	f.write(LdmrkAngs)
if(np.save('testfile'+str(NLdmrk)+'.npy',LdmrkAngs)):
	print 'aakash'


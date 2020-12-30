import numpy as np

class AcousticReciever():
    def __init__(self,pos,detection_range):
	    self.range=detection_range
	    self.pos = pos
	    self.detectionList=[]
	    self.detectionSet=set()
		
    def detected(self,tag,time,bin,dim=3):
	    detected=False
	    pos=self.pos
	    if dim ==2:
	        pos = self.pos[0:2]
	    #detection_model = self.range*np.random.rand()
	    detection_model = self.range
	    if np.linalg.norm(pos-tag.pos[:dim])<detection_model:
	        detected =tag.pinging(time)
	        #print(np.linalg.norm(pos-tag.pos[:dim]),self.range,tag.ID,detected,time,tag.last_ping,time-tag.last_ping)
	    if detected:
	        self.detectionList.append((tag.ID,time,self.pos,bin))
	        self.detectionSet.add((tag.ID))
	        #print('detected',len(self.detectionList),len(self.detectionSet))
	    return detected
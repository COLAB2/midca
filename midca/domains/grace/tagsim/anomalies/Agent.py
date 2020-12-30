import numpy as np


def m1_step(x,u):
        return 1*np.array([np.cos(u), np.sin(u)])
      
def m2_step(x,u):
	return 0.005*np.array([-0.5*np.cos(u), np.sin(u)])
	
class Agent():
    def  __init__(self,state,sensor,world=None,x_index=0,y_index=1,z_index=2,dim=3):
        self.state = state
        self.dynamics=None
        self.sensor=sensor
        self.pos_idx=[x_index,y_index,z_index]
        self.world = world
        self.belief_map =  np.zeros_like(world.p)
        self.belief_count=np.zeros_like(world.p)
        if dim ==2:
            self.pos_idx=[x_index,y_index]
        self.dim = dim  
        sensor.pos=self.getPos()
		# starting abstracted position
        self.s = 0
		# starting mode
        self.m = 0
        # mode switch probability
        self.m_p = 0.0
		
    def updateAgent(self,state,time,dets=[]):
        self.state=state
        pos=self.getPos()
        self.sensor.pos=pos
        bin=self.world.getAbstractPos(pos[0],pos[1])
        detections=False
        for tag in dets:
            #self.sensor.detected(tag,time,bin)
            self.sensor.detectionList.append((tag,time,pos,bin))
            self.sensor.detectionSet.add((tag))
        return detections
		    
    def getPos(self):
        pos_idx=self.pos_idx
        if self.dim == 3:
            return np.array([self.state[pos_idx[0]],self.state[pos_idx[1]],self.state[pos_idx[2]]])
        return np.array([self.state[pos_idx[0]],self.state[pos_idx[1]]])
      
    

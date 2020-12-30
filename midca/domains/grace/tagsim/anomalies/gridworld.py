import numpy as np

''' Gridworld simulation:
    | 20 | 21 | 22 | 23 | 24 |
    | 15 | 16 | 17 | 18 | 19 |
    | 10 | 11 | 12 | 13 | 14 |
    | 5  | 6  | 7  | 8  | 9  |
    | 0  | 1  | 2  | 3  | 4  |

    - true position: x in [0, 5] x [0, 5] 
    - abstracted position: s in {0, 1, ..., 24}
    - true input: u in [-pi, pi]

    Grid centers:
    - 0: (0.5, 0.5)
    - 1: (1.5, 0.5)
    - 2: (2.5, 0.5)
    - 3: (3.5, 0.5)
    - 4: (4.5, 0.5)

    - 5: (0.5, 1.5)
    - 6: (1.5, 1.5)
    - 7: (2.5, 1.5)
    - 8: (3.5, 1.5)
    - 9: (4.5, 1.5)

    - 10: (0.5, 2.5)
    - 11: (1.5, 2.5)
    - 12: (2.5, 2.5)
    - 13: (3.5, 2.5)
    - 14: (4.5, 2.5)

    - 15: (0.5, 3.5)
    - 16: (1.5, 3.5)
    - 17: (2.5, 3.5)
    - 18: (3.5, 3.5)
    - 19: (4.5, .5)

    - 20: (0.5, 4.5)
    - 21: (1.5, 4.5)
    - 22: (2.5, 4.5)
    - 23: (3.5, 4.5)
    - 24: (4.5, 4.5)
'''


	
def iterative_average(x,n,ave):
	return ave+(x-ave)/(n+1)


class Grid:
    def __init__(self,taglist,x_range=1000.0,y_range=1000.0):
        self.taglist=taglist
        self.np_taglist=None
        self.x_range = x_range
        self.y_range = y_range
        self.x_bins = np.array([0., 1., 2., 3., 4.])*self.x_range/5.0
        self.y_bins = np.array([0., 1., 2., 3., 4.])*self.y_range/5.0

        # specifies tag density for each cell (hotspot probabilities?)
        self.p = np.array([0.1, 0.1, 0.4, 0.6, 0.2,
                           0.1, 0.3, 0.5, 0.8, 0.3,
                           0.2, 0.4, 0.2, 0.2, 0.1,
                           0.5, 0.9, 0.4, 0.2, 0.1,
                           0.3, 0.6, 0.3, 0.1, 0.1])
        self.tag_count = np.zeros_like(self.p) 

    def state(self,Agent):
        pos=Agent.getPos()
        return self.getAbstractPos(pos[0],pos[1])
    
    def saveTagList(self,fname="tags"):
        f=open(fname+".csv",'+w')
        for tag in self.taglist:
            last_ping=tag.last_ping
            posx=tag.pos[0]
            posy=tag.pos[1]
            posz=tag.pos[2]
            delay=tag.delay
            ID=tag.ID
            bin=tag.bin
            f.write("{0},{1},{2},{3},{4},{5},{6}\n".format(last_ping,posx,posy,posz,delay,ID,bin))
        f.close()
	
    def tagListToNumpyArray(self):
        a=np.array()
        for tag in self.taglist:
            last_ping=tag.last_ping
            posx=tag.pos[0]
            posy=tag.pos[1]
            posz=tag.pos[2]
            delay=tag.delay
            ID=tag.ID
            bin=tag.bin
            a.append([last_ping,posx,posy,posz,delay,ID,bin])
        return a
		
    def nploadTagList(self,fname="tags"):
        return np.genfromtxt(fname+".csv",delimiter=",")
		
    def loadTagList(self,fname="tags"):
        f=open(fname+".csv",'r')
        taglist=[]
        from AcousticTag import AcousticTag
        for line in f.readlines():
            last_ping,posx,posy,posz,delay,ID,bin=line.split(',')
            tag = AcousticTag(int(ID),last_ping=float(last_ping),ping_delay=float(delay))
            tag.pos=np.array([float(posx),float(posy),float(posz)])
            tag.bin=bin
            taglist.append(tag)
        f.close()
        self.taglist=taglist
        return taglist
		
    def act(self,agent, u,dt):
        bins = [0., 1., 2., 3., 4.]

        # simulate forward with inputs
        p = 0*u.copy()
        for t in range(len(u)):
            # f0 dynamics
            if agent.m == 0:
                self.x += self.flow(self.x) +agent.m1_step(u[t],dt)

            # f1 dynamics
            elif agent.m == 1:
                self.x += self.flow(self.x) + agent.m2_step(u[t],dt)

            # randomly switch modes
            if np.random.rand() < agent.m_p:
                agent.m = 1

		
    def getAbstractPos(self,x,y):
        idx = np.digitize(x, self.x_bins)
        idy = np.digitize(y, self.y_bins)
        return 5*(idy-1) + (idx)
		
		
    def abstractPosToRange(self,s):
        return (s%len(self.x_bins))*self.x_range*1.0/len(self.x_bins),((s-s%len(self.x_bins))*1.0/len(self.x_bins))%len(self.y_bins)*self.y_range/len(self.y_bins),20
	
    def getCellXY(self,x,y):
        idx = np.digitize(x, self.x_bins)
        idy = np.digitize(y, self.y_bins)
        return idx,idy
		
    def groundTruth(self,tau,fixed=False,time_step=1):
        bin_rates=np.zeros_like(self.p)
        bin_counts=0
        bin_lens=np.zeros_like(self.p)
        bin_sets=[set()]*len(bin_rates)
        N = len(self.taglist)
        time_step=min(tau/2.0,time_step)
        t=0
        last_meas=0
        max_delay=0
        for i in range(N):
            if self.taglist[i].delay > max_delay:
                max_delay=self.taglist[i].delay
            if fixed:
                self.taglist[i].last_ping=0
            else:
                self.taglist[i].last_ping=np.random.randn()
        if time_step > max_delay:
            print('time step longer than max ping rate. results may be in accurate')
        while t<=200*tau or t<max_delay*10: 
            for tag in self.taglist:
                if tag.pinging(t):
                    if tag.ID not in bin_sets[tag.bin]:
                        bin_sets[tag.bin].add(tag.ID)
                        bin_lens[tag.bin]+=1
       
                tag.updatePing(t)
            if last_meas+tau<=t:
                rate_meas = bin_lens/tau
                bin_counts+=1
                bin_rates= iterative_average(rate_meas,bin_counts,bin_rates)  #iteratively average rate measurement
                last_meas=t
                bin_sets=[set()]*len(bin_rates)
                bin_lens=0*bin_lens
            t+=time_step
        bin_rates.shape=(5,5)
        return np.flip(bin_rates,0)
	
    def approximateField(self,tau,fixed=False,time_step=1,spacing=None,sensorRange=1,dim=2,get_points=False):
        if type(spacing) == type(None): #spacinging ( xunits, yunits)
            return self.groundTruth(tau,fixed,time_step)
        from AcousticReciever import AcousticReciever
        nx_bins = int(self.x_range/spacing[0])
        ny_bins = int(self.y_range/spacing[1])
        bin_rates=np.zeros((nx_bins,ny_bins))
        x_bins=np.array(range(nx_bins))*self.x_range/nx_bins
        y_bins=np.array(range(ny_bins))*self.y_range/ny_bins
       
        bin_counts=0
        bin_lens=np.zeros_like(bin_rates)
        bin_sets=[set() for i in range((nx_bins*ny_bins))]
        N = len(self.taglist)
        time_step=min(tau/2.0,time_step)
		#create acoustic sensor for each bin with center as its location
        points = [(idx+spacing[0]/2.0,idy+spacing[1]/2.0) for idx in x_bins for idy in y_bins]
        points =  np.array(points)
        #print(points)
		#get tags in range of each sensor
        inrange=[set() for i in range((nx_bins*ny_bins))]
        for tag in self.taglist:
            diff=points-tag.pos[:2]
            dets=np.sqrt(diff[:,0]*diff[:,0]+diff[:,1]*diff[:,1])<sensorRange
            dets.shape=(len(dets),1)
            #print(dets,points,np.sqrt(diff[:,0]*diff[:,0]+diff[:,1]*diff[:,1]))
            #print(tag.pos,tag.ID)
            #input()
            pos=points[np.where(dets==1)[0],:]
            for px,py in pos:
              idx = np.digitize(px, x_bins)
              idy = np.digitize(py, y_bins)
              inrange[nx_bins*(idy-1) + (idx)-1].add(tag.ID)
              #print(inrange[nx_bins*(idy-1) + (idx)-1],px,py,nx_bins*(idy-1) + (idx)-1)
        #print(inrange[0],inrange[1],inrange[2],inrange[3])
        t=0 
        last_meas=0
        max_delay=0
        for i in range(N):
            if self.taglist[i].delay > max_delay:
                max_delay=self.taglist[i].delay
            if fixed:
                self.taglist[i].last_ping=0
            else:
                self.taglist[i].last_ping=np.random.randn()
        if time_step > max_delay:
            print('time step longer than max ping rate. results may be in accurate')
        while t<=200*tau or t<max_delay*10: 
            for tag in self.taglist:
                if tag.pinging(t):
                    for i in range(len(inrange)):
                        #print(inrange[i],i)
                        if tag.ID in inrange[i] and tag.ID not in bin_sets[i]:
                            px,py=points[i,:]
                            idx = np.digitize(px, x_bins)-1
                            idy = np.digitize(py, y_bins)-1
                            bin_sets[i].add(tag.ID)
                            bin_lens[idx,idy]+=1 
                    #input()
                tag.updatePing(t)
            if last_meas+tau<=t:
                rate_meas = bin_lens/tau
                bin_counts+=1
                bin_rates= iterative_average(rate_meas,bin_counts,bin_rates)  #iteratively average rate measurement
                last_meas=t
                bin_sets=[set() for i in range((nx_bins*ny_bins))]
                bin_lens=0*bin_lens
				
            t+=time_step
        if get_points:
            points.shape=(nx_bins,ny_bins,2)
            return np.flip(bin_rates,0),np.flip(points,0)
        return np.flip(bin_rates,0)
		
    def approximateFieldMax(self,tau,fixed=False,time_step=1,spacing=None,sensorRange=1,dim=2,get_points=False):
        if type(spacing) == type(None): #spacinging ( xunits, yunits)
            return self.groundTruth(tau,fixed,time_step)
        from AcousticReciever import AcousticReciever
        nx_bins = int(self.x_range/spacing[0])
        ny_bins = int(self.y_range/spacing[1])
        bin_rates=np.ones((nx_bins,ny_bins))*0.0
        x_bins=np.array(range(nx_bins))*self.x_range/nx_bins
        y_bins=np.array(range(ny_bins))*self.y_range/ny_bins
       
        bin_counts=0
        bin_lens=np.zeros_like(bin_rates)
        bin_sets=[set() for i in range((nx_bins*ny_bins))]
        N = len(self.taglist)
        time_step=min(tau/2.0,time_step)
		#create acoustic sensor for each bin with center as its location
        points = [(idx+spacing[0]/2.0,idy+spacing[1]/2.0) for idx in x_bins for idy in y_bins]
        points =  np.array(points)
        #print(points)
		#get tags in range of each sensor
        inrange=[set() for i in range((nx_bins*ny_bins))]
        offset=0
        for tag in self.taglist:
            if tag.last_ping>offset:
                offset=tag.last_ping
            diff=points-tag.pos[:2]
            dets=np.sqrt(diff[:,0]*diff[:,0]+diff[:,1]*diff[:,1])<sensorRange
            dets.shape=(len(dets),1)
            #print(dets,points,np.sqrt(diff[:,0]*diff[:,0]+diff[:,1]*diff[:,1]))
            #print(tag.pos,tag.ID)
            #input()
            pos=points[np.where(dets==1)[0],:]
            for px,py in pos:
              idx = np.digitize(px, x_bins)
              idy = np.digitize(py, y_bins)
              inrange[nx_bins*(idy-1) + (idx)-1].add(tag.ID)
              #print(inrange[nx_bins*(idy-1) + (idx)-1],px,py,nx_bins*(idy-1) + (idx)-1)
        #print(inrange[0],inrange[1],inrange[2],inrange[3])
        t=0 
        last_meas=0
        max_delay=0
        for i in range(N):
            if self.taglist[i].delay > max_delay:
                max_delay=self.taglist[i].delay
            if fixed:
                self.taglist[i].last_ping=self.taglist[i].last_ping-offset
            else:
                self.taglist[i].last_ping=np.random.randn()
        if time_step > max_delay:
            print('time step longer than max ping rate. results may be in accurate')
        while t<=200*tau or t<max_delay*10: 
            for tag in self.taglist:
                if tag.pinging(t):
                    for i in range(len(inrange)):
                        #print(inrange[i],i)
                        if tag.ID in inrange[i] and tag.ID not in bin_sets[i]:
                            px,py=points[i,:]
                            idx = np.digitize(px, x_bins)-1
                            idy = np.digitize(py, y_bins)-1
                            bin_sets[i].add(tag.ID)
                            bin_lens[idx,idy]+=1 
                    #input()
                tag.updatePing(t)
            if last_meas+tau<=t:
                rate_meas = bin_lens/tau
                bin_counts+=1
                bin_rates=np.where(bin_rates<rate_meas,rate_meas,bin_rates)
                last_meas=t
                bin_sets=[set() for i in range((nx_bins*ny_bins))]
                bin_lens=0*bin_lens
				
            t+=time_step
        if get_points:
            points.shape=(nx_bins,ny_bins,2)
            return np.flip(bin_rates,0),np.flip(points,0)
        return np.flip(bin_rates,0)	
		
    def flow(self, x):
        return 0.004*np.array([np.cos(0.25*x[0]), np.sin(0.6*x[1])])
		
    def setMap(self,map=None):
        if type(map)==type(None):
            self.p=np.array([0.1, 0.1, 0.4, 0.6, 0.2,
                           0.1, 0.3, 0.5, 0.8, 0.3,
                           0.2, 0.4, 0.2, 0.2, 0.1,
                           0.5, 0.9, 0.4, 0.2, 0.1,
                           0.3, 0.6, 0.3, 0.1, 0.1])
        else:
            self.p = map
        count = 0
        self.tag_count = np.zeros_like(self.p) 
        N=len(self.taglist)
        while count < N:
            add_prob = np.random.rand()
            to_add = add_prob<self.p
            for i in range(self.p.size):
                if to_add[i] and count<N:
                    xr,yr,zr = self.abstractPosToRange(i)
                    mpx,mpy =(self.x_range*1.0/len(self.x_bins),self.y_range*1.0/len(self.y_bins))
                    self.taglist[count].pos=np.array([xr+mpx*np.random.rand(),yr+mpy*np.random.rand(),zr*np.random.rand()])
                    self.taglist[count].bin=i
                    self.tag_count[i]+=1
                    count+=1
				    
				    
				    
if __name__ == '__main__':
	from AcousticTag import AcousticTag
	taglist=[]
	N=1000
	for i in range(N):
		taglist.append(AcousticTag(i))
	grid = Grid(taglist)
	grid.setMap()
	tau=.5
	# print('tau=',tau)
	# print(np.round(grid.groundTruth(tau),decimals=2))
	# print('method 2 tau=',tau)
	# print(np.round(grid.approximateField(tau,spacing=(200,200),sensorRange=110),decimals=2))
	# #print(np.round(grid.groundTruth(tau,fixed=True),decimals=2))
	# tau=2
	# print('tau=',tau)
	# print(np.round(grid.groundTruth(tau),decimals=2))
	# print('method 2 tau=',tau)
	# print(np.round(grid.approximateField(tau,spacing=(200,200),sensorRange=110),decimals=2))
	# #print(np.round(grid.groundTruth(tau,fixed=True),decimals=2))
	# tau=5
	# print('tau=',tau)
	# print(np.round(grid.groundTruth(tau),decimals=2))
	# print('method 2 tau=',tau)
	# print(np.round(grid.approximateField(tau,spacing=(200,200),sensorRange=100),decimals=2))
	# #print(np.round(grid.groundTruth(tau,fixed=True),decimals=2))
	# tau=10
	# print('tau=',tau)
	# print(np.round(grid.groundTruth(tau),decimals=2))
	# print('method 2 tau=',tau)
	# print(np.round(grid.approximateField(tau,spacing=(200,200),sensorRange=100),decimals=2))
	# #print(np.round(grid.groundTruth(tau,fixed=True),decimals=2))
	# tau=17
	# print('tau=',tau)
	# print(np.round(grid.groundTruth(tau,time_step=5),decimals=2))
	# print('method 2 tau=',tau)
	# print(np.round(grid.approximateField(tau,spacing=(200,200),sensorRange=100),decimals=2))
	# #print(np.round(grid.groundTruth(tau,fixed=True,time_step=5),decimals=2))
	# tau=34
	# print('tau=',tau)
	# print(np.round(grid.groundTruth(tau,time_step=5),decimals=2))
	# #print(np.round(grid.groundTruth(tau,fixed=True,time_step=5),decimals=2))
	# print('method 2 tau=',tau)
	# print(np.round(grid.approximateField(tau,spacing=(200,200),sensorRange=100),decimals=2))
	tau=2
	print(np.round(grid.approximateField(tau,spacing=(200,200),sensorRange=50),decimals=2))
	

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


    def saveTagList(self,fname="tags"):
        f=open(fname+".csv",'w')
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

    def loadTagList(self,fname="tags"):
        f=open(fname+".csv",'r')
        taglist=[]
        from AcousticTag import AcousticTag
        for line in f.readlines():
            last_ping,posx,posy,posz,delay,ID,bin=line.split(',')
            tag = AcousticTag(ID,last_ping=float(last_ping),ping_delay=float(delay))
            tag.pos=np.array([float(posx),float(posy),float(posz)])
            tag.bin=bin
            taglist.append(tag)
        f.close()
        self.taglist=taglist
        return taglist


    #def abstractPosToRange(self,s):
    #    return (s%len(self.x_bins))*self.x_range*1.0/len(self.x_bins),((s-s%len(self.x_bins))*1.0/len(self.x_bins))%len(self.y_bins)*self.y_range/len(self.y_bins),20

    def abstractPosToRange(self,s):
        start_x = -141
        start_y = 76 - 80*5
        side = 80
        tags = []
        grid_n = 5
        grid_m = 5
        for i in range(grid_n):
            for j in range(grid_m):
                tags.append([start_x + j*80, start_y + i*80])

        return tags[s][0], tags[s][1], 20


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
                    mpx,mpy = 80,80
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

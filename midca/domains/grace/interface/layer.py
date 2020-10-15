from gridworld import Grid
from AcousticTag import AcousticTag
from mine_layer import Minelayer
import numpy as np

np.random.seed(555)

def generate_tags(N=1000):

    #initializations
    taglist=[]
    for i in range(N):
    	#taglist.append(AcousticTag(i,last_ping=30*np.random.randn()),ping_delay=max(2,30*np.random.randn())) # most realistic
    	taglist.append(AcousticTag(i,last_ping=15*np.random.randn())) # more realistic (pings are not aligned in time)
    	#taglist.append(AcousticTag(i)) #better for understanding because pings are aligned in time and  all have same ping interval

    density_map = np.array([0.1, 0.1, 0.4, 0.3, 0.2,
    			0.1, 0.3, 0.3, 0.1, 0.3,
    			0.2, 0.3, 0.3, 0.2, 0.1,
    			0.3, 0.9, 0.3, 0.2, 0.1,
    			0.2, 0.3, 0.2, 0.1, 0.1])

    E = Grid(taglist)
    E.setMap(density_map)

    return [(tag.pos[0], tag.pos[1]) for tag in E.taglist]

# generate tag positions
taglist = generate_tags(1000)

# create mine layer object
minelayer = Minelayer()

# lay mines in moos
minelayer.write_to_file(taglist)

import numpy as np
import matplotlib.pyplot as plt
from gridworld import Grid
import gridworld
from AcousticTag import AcousticTag
from AcousticReciever import AcousticReciever



taglist=[]
E = Grid(taglist,x_range=100, y_range=100)
taglist=E.loadTagList(fname="testField")#load as list of tags
tagData=E.nploadTagList(fname="testField")#load tagData into numpy array






############################################## Plotting ############################
def draw(x,fig=1): #function for plotting field
	plt.figure(fig)
	plt.axis('scaled')
	plt.grid(True)
	plt.plot(x[0], x[1], 'r.')
	plt.xlim([0, E.x_range])
	plt.ylim([0, E.y_range])
	plt.xticks(np.arange(0,E.x_range,E.x_range/5.0))
	plt.yticks(np.arange(0,E.y_range,E.y_range/5.0))
    
	plt.draw()

#get all tag position to plot
N=len(taglist)
tagx=np.zeros(N)
tagy=np.zeros(N)
for i in range(N):
	x,y,_ = taglist[i].pos
	tagx[i]=x
	tagy[i]=y
draw((tagx,tagy))
plt.title("from taglist")
plt.pause(0.00001)

posx=tagData[:,1]
posy=tagData[:,2]
draw((posx,posy),fig=2)
plt.title("from tagData")
plt.pause(0.00001)

input("press enter to end")
import numpy as np
import matplotlib.pyplot as plt
from gridworld import Grid
import gridworld
from AcousticTag import AcousticTag
from AcousticReciever import AcousticReciever

def draw(x,fig=1): #function for plotting field
	plt.figure(fig)
	plt.axis('scaled')
	plt.grid(True)
	plt.plot(x[0], x[1], 'r.')
	plt.xlim([0, grid.x_range])
	plt.ylim([0, grid.y_range])
	plt.xticks(np.arange(0,grid.x_range,grid.x_range/5.0))
	plt.yticks(np.arange(0,grid.y_range,grid.y_range/5.0))
    
	plt.draw()

taglist=[]
N=100# number of tags to be generated in field
#generate N tags and append to taglist
for i in range(N):
	delay=max(1,30*np.random.rand())
	#each tag object has an ID, an offset (last_ping), and a delay between pings (ping_delay)
	taglist.append(AcousticTag(i,last_ping=-delay*np.random.rand(),ping_delay=delay))
# create grid world object
grid = Grid(taglist,x_range=100, y_range=100)
#make map of densities for each cell
density_map = np.array([0.1, 0.1, 0.4, 0.3, 0.2,
						0.1, 0.3, 0.3, 0.1, 0.3,
						0.2, 0.3, 0.3, 0.2, 0.1,
						0.3, 0.9, 0.3, 0.2, 0.1,
						0.2, 0.3, 0.2, 0.1, 0.1])
grid.setMap(density_map)#set map with no arguement sets the default map
#saveTaglist to .csv file 
grid.saveTagList(fname="testField")
############################################### Plotting ############################


#get all tag position to plot
tagx=np.zeros(N)
tagy=np.zeros(N)
for i in range(len(taglist)):
	x,y,_ = taglist[i].pos
	tagx[i]=x
	tagy[i]=y
draw((tagx,tagy))
plt.title("all tags")
plt.pause(0.00001)


#get only tag position of pinging tags to plot
t=1
pinging_x =np.zeros(1)
pinging_y =np.zeros(1)
for tag in taglist:
	if tag.pinging(t):
		x,y,_ = tag.pos
		pinging_x=np.append(pinging_x,x)
		pinging_y=np.append(pinging_y,y)
	tag.updatePing(t)#if simulating, we need to update the tags based on time
draw((pinging_x,pinging_y),fig=2)	
plt.title("tags ping at time t="+str(t))
plt.pause(0.00001)


#plot detection rate for a specific settings
grid.p.shape=(5,5)
print(np.flip(grid.p,0))
spacing=(.05*grid.x_range,.05*grid.y_range)
sensorRange=.1*grid.x_range
measurement_time=2
print("Rate field approximation for sensor with range",sensorRange," spaced at intervals of",spacing)
approx,pnts=grid.approximateField(measurement_time,spacing=spacing,sensorRange=sensorRange,get_points=True)
plt.figure(3)
plt.axis('scaled')
plt.grid(True)
plt.contourf(pnts[:,:,0], pnts[:,:,1], np.flip(np.round(approx,decimals=2),(0,1)).transpose(), 20, cmap='coolwarm')# cmap='inferno'), cmap='RdGy')
cbar = plt.colorbar()
cbar.set_label('Detection rate')
plt.xlim([0, grid.x_range])
plt.ylim([0, grid.y_range])
plt.xticks(np.arange(0,grid.x_range,grid.x_range/5.0))
plt.yticks(np.arange(0,grid.y_range,grid.y_range/5.0))
plt.pause(0.00001)
input("press enter to end")
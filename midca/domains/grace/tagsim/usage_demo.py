import numpy as np
import matplotlib.pyplot as plt
from gridworld import Grid
from AcousticTag import AcousticTag
from Agent import Agent
from AcousticReciever import AcousticReciever
'''
psuedo code
create grid world
generate tag positons based on probability map and total number of fish N
create agents

simulation:
  1. detect tags
  2. update tag.lastPing
  3. updte agent dynamics

'''
def draw(x):
    plt.figure(1)
    plt.axis('scaled')
    plt.grid(True)
    plt.plot(x[0], x[1], 'r.')
    plt.xlim([0, E.x_range])
    plt.ylim([0, E.y_range])
    plt.xticks(np.arange(0,E.x_range,E.x_range/5.0))
    plt.yticks(np.arange(0,E.y_range,E.y_range/5.0))
    
    plt.draw()
    plt.pause(0.0001)

def drawAgent(x):
    plt.figure(1)
    plt.axis('scaled')
    plt.grid(True)
    plt.plot(x[0], x[1], 'bo')
    plt.xlim([0, E.x_range])
    plt.ylim([0, E.y_range])
    plt.xticks(np.arange(0,E.x_range,E.x_range/5.0))
    plt.yticks(np.arange(0,E.y_range,E.y_range/5.0))
    
    plt.draw()
    plt.pause(0.0001)
	
def simulate_dynamics(agent,u,tspan,dt):
	inc = agent.state
	for i in np.linspace(tspan[0],tspan[1],int((tspan[1]-tspan[0])/dt)):
		inc+=agent.dynamics(inc,u)*dt#+world.flow(agent.getPos())*dt
	return inc
	
def f1_plan(x0, x, N):
    u = np.zeros(N)
    xsim = x0.copy()
    for n in range(N):
        e = x-xsim
        angle = np.arctan2(e[1], e[0])
        u[n] = angle
        xsim += 0.005*np.array([np.cos(u[n]), np.sin(u[n])])

    return u

def m1_step(x,u):
        return 1*np.array([np.cos(u), np.sin(u)])
	
	#settings
N = 1000 #how many tags present
simtime=180 #max simulation time
numAgents=1 #number of agents exploring
time_step=.5 
fileToLoad =""#"tags"
show_only_when_pinging = True
     #setup
taglist=[]
agentList=[]
tagx=np.zeros(N)
tagy=np.zeros(N)
for i in range(N):
	#taglist.append(AcousticTag(i,last_ping=np.random.randn()),ping_delay=max(2,30*np.random.randn())) # most realistic 
    taglist.append(AcousticTag(i,last_ping=10*np.random.randn())) # more realistic (pings are not aligned in time)
    #taglist.append(AcousticTag(i)) #better for understanding because pings are aligned in time and  all have same ping interval
    x,y,_ = taglist[i].pos
    tagx[i]=x
    tagy[i]=y
	
#E = Grid(taglist,x_range=20,y_range=20)
E = Grid(taglist)
if fileToLoad=="":
    E.setMap()
    #E.saveTagList()
else:
    taglist=E.loadTagList()
	
for i in range(numAgents):
    s= AcousticReciever(np.array([0,0,0]),50)
    agentList.append(Agent(np.array([(i+1)*E.x_range/5,3*E.y_range/5]),s,E,dim=2))
    agentList[i].dynamics=m1_step
	
for i in range(N):
    x,y,_ = taglist[i].pos
    tagx[i]=x
    tagy[i]=y
	

#draw((tagx,tagy))
# plt.figure(3)
# plt.axis('scaled')
# plt.grid(True)
# plt.plot(tagx, tagy, 'r.')
# plt.xlim([0, 1000])
# plt.ylim([0, 1000])
# plt.xticks(np.arange(0,1000,200))
# plt.yticks(np.arange(0,1000,200))

# plt.draw()
# plt.pause(0.0001)
# plt.figure(1)
'''
for t in range(N):
    draw(taglist[t].pos)
   #plt.pause(.1)
''' 
        # simulation
#input('Enter to begin simulation')
t=0
while t<=simtime: #change to better simulation stopping criteria 
	posx=np.zeros(numAgents)
	posy=np.zeros(numAgents)
	pinging_x =np.zeros(1)
	pinging_y =np.zeros(1)
	for i in range(len(agentList)):
		agent=agentList[i]
		#state = agent.getPos()#
		state=simulate_dynamics(agent,.3, [0,time_step],.1)
		dets=agent.updateAgent(state,t)
		if dets>0:
			print(dets,' detections',t,state)
		pos=agent.getPos()
		posx[i]=pos[0]
		posy[i]=pos[1]
	for tag in taglist:
		if tag.pinging(t) and show_only_when_pinging:
			x,y,_ = tag.pos
			pinging_x=np.append(pinging_x,x)
			pinging_y=np.append(pinging_y,y)
		tag.updatePing(t)
	t+=time_step
	plt.clf()
	if show_only_when_pinging:
		draw((pinging_x,pinging_y))
	else:
		pdraw((tagx,tagy))
	drawAgent((posx,posy))
	plt.pause(.001)
	
draw((tagx,tagy))	
drawAgent((posx,posy))
print(agent.sensor.detectionSet)
print('detections:',len(agent.sensor.detectionList),', unique detections:',len(agent.sensor.detectionSet),)
#print(agent.sensor.detectionList)

print("True probability density  map")
E.p.shape=(5,5)
print(np.flip(E.p,0))
spacing=(50,50)
#spacing=(25,25)
#spacing=(1,1)
sensorRange=50
measurement_time=3
print("Rate field approximation for sensor with range",sensorRange," spaced at intervals of",spacing)
approx,pnts=E.approximateField(measurement_time,spacing=spacing,sensorRange=sensorRange,get_points=True)
#print(np.round(approx,decimals=2))
plt.figure(2)
plt.axis('scaled')
plt.grid(True)
#print('\n',pnts[:,:,0],'\n',pnts[:,:,1])
#plt.plot(pnts[:,:,0].flatten(), pnts[:,:,1].flatten(), 'r.',cmap='coolwarm')
plt.contourf(pnts[:,:,0], pnts[:,:,1], np.flip(np.round(approx,decimals=2),(0,1)).transpose(), 20, cmap='coolwarm')# cmap='inferno'), cmap='RdGy')


cbar = plt.colorbar()
cbar.set_label('Detection rate')

plt.xlim([0, E.x_range])
plt.ylim([0, E.y_range])
plt.xticks(np.arange(0,E.x_range,spacing[0]))
plt.yticks(np.arange(0,E.y_range,spacing[1]))
plt.draw()
plt.pause(0.00001)
input('done')



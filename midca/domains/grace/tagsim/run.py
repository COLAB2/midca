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
    plt.xlim([0, 1000])
    plt.ylim([0, 1000])
    plt.xticks(np.arange(0,1000,200))
    plt.yticks(np.arange(0,1000,200))
    
    plt.draw()
    plt.pause(0.0001)

def drawAgent(x):
    plt.figure(1)
    plt.axis('scaled')
    plt.grid(True)
    plt.plot(x[0], x[1], 'bo')
    plt.xlim([0, 1000])
    plt.ylim([0, 1000])
    plt.xticks(np.arange(0,1000,200))
    plt.yticks(np.arange(0,1000,200))
    
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
N = 1000
simtime=3
numAgents=1
time_step=.5
#np.random.seed(1)
     #setup
taglist=[]
agentList=[]
tagx=np.zeros(N)
tagy=np.zeros(N)
for i in range(N):
    taglist.append(AcousticTag(i,last_ping=15*np.random.randn())) # more realistic
    #taglist.append(AcousticTag(i)) #better for understanding
    x,y,_ = taglist[i].pos
    tagx[i]=x
    tagy[i]=y
	
E = Grid(taglist)
E.setMap()

for i in range(numAgents):
    s= AcousticReciever(np.array([0,0,0]),50)
    agentList.append(Agent(np.array([50.0,600.0]),s,E,dim=2))
    agentList[i].dynamics=m1_step
	
for i in range(N):
    x,y,_ = taglist[i].pos
    tagx[i]=x
    tagy[i]=y
	

draw((tagx,tagy))
'''
for t in range(N):
    draw(taglist[t].pos)
   #plt.pause(.1)
''' 
        # simulation
#input('Enter to begin simulation')
t=0
while t<=simtime:
	posx=np.zeros(numAgents)
	posy=np.zeros(numAgents)
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
		tag.updatePing(t)
	t+=time_step
	plt.clf()
	draw((tagx,tagy))
	drawAgent((posx,posy))
	plt.pause(.001)
	
	
print(agent.sensor.detectionSet)
print('detections:',len(agent.sensor.detectionList),', unique detections:',len(agent.sensor.detectionSet),)

#print(agent.sensor.detectionList)
input('done')



